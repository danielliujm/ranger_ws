import torch
from pytorch_mppi import MPPI, KMPPI, mppi
from alpaca_navigation.mppi_config import *
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.vectorized import contains
from nav_msgs.msg import OccupancyGrid
import math
from alpaca_navigation.cv import *


class Costmap: 
    def __init__ (self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_theta = 2.0 * math.atan2(msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        self.data = np.array(msg.data).reshape((self.height, self.width))



class SMMPPIController:
    def __init__(self,static_obs, device):


        # Initialize parameters from config
        self.horizon = HORIZON_LENGTH 
        self.dt = DT
        self.device = device
        self.angular_alignment_threshold = ANGULAR_THRESHOLD   # Angular error threshold in radians
        self.goals = torch.tensor(GOALS, dtype=torch.float32).to(self.device)
        self.rollouts = torch.zeros((7, NUM_SAMPLES, 2))
        self.costs = torch.zeros((7, NUM_SAMPLES, 2))
        self.max_cycles = NUM_CYCLES
        self.num_samples = NUM_SAMPLES
        self.polygons = [Polygon(obs) for obs in static_obs]
        self.multi_polygon = MultiPolygon(self.polygons)
        self.bounds = self.multi_polygon.bounds
        self.s2_ego = torch.zeros((self.num_samples, 3)).to(self.device)
        self.sigma_h = SIGMA_H
        self.sigma_s = SIGMA_S
        self.sigma_r = SIGMA_R
        self.q_obs = Q_OBS


        self.current_goal_index = 0
        self.cycle_count = 0
        self.counter = 0
        self.goal = torch.tensor ([0,0]).to(self.device)
        self.agent_weights = {i: torch.tensor([0.1, 0.1, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}

        self.interacting_agents = []
        self.local_costmap = None

        # Initialize MPPI with the dynamics and cost functions
        cov = torch.eye(3, dtype=torch.float32).to(self.device)
        cov[0, 0] = 0.02 #0.001
        cov[1, 1] = 10e-8 #0.001
        cov[2, 2] = 0.02 #0.001
        # MPPI initialization
        # print (f'horizon lenghth is {self.horizon} ')
        U_init = torch.zeros((self.horizon, 3)).to(self.device)
        U_init [:,0] = 0.4

        self.mppi = mppi.KMPPI(
            self.dynamics,
            self.cost,
            3,  # State dimension
            cov,
            num_samples=self.num_samples,
            horizon=self.horizon,
            device=self.device,
            terminal_state_cost=self.terminal_cost,
            step_dependent_dynamics=True,


            u_min=torch.tensor([0.0, -0.0, -1.5], dtype=torch.float32).to(self.device),
            u_max=torch.tensor([0.6, 0.0, 1.5], dtype=torch.float32).to(self.device),
            
            lambda_ = 1e-2,
            kernel = mppi.RBFKernel (sigma = 2.0),
            num_support_pts=self.horizon//10,
            # w_action_seq_cost=10,

           

            U_init = U_init
        )
    
    def set_goal (self, goal):
        self.goal = goal.to(self.device)

    
    def set_local_costmap (self, msg):
        self.local_costmap = Costmap (msg)
        


    def compute_control(self, current_state, previous_robot_state, robot_velocity, agent_states, previous_agent_states,agent_velocities): 
        self.current_state = current_state
        self.previous_robot_state = previous_robot_state
        self.robot_velocity = robot_velocity
        
        self.agent_states = agent_states
        self.previous_agent_states = previous_agent_states
        self.agent_velocities = agent_velocities
        action = self.mppi.command(current_state)
        self.mppi.u_init = action
        rollouts = self.mppi.get_rollouts (current_state, num_rollouts = 1)
        costs = self.mppi.cost_total.squeeze(0)

        termination =  torch.linalg.norm(self.current_state[:2] - self.goal) < TERMINATION_TOLERANCE
        
        return action, rollouts, costs, termination

    def cost(self, state: torch.Tensor, action: torch.Tensor, t) -> torch.Tensor:
        """
        Cost function for MPPI optimization.
        Args:
            state: (num_samples, 3) - States over the horizon.
            action: (num_samples, 3) - Actions over the horizon.

        Returns:
            cost: (num_samples) - Total cost for each sample.
        """

        

       

        # cost = heading_cost # + action_cost
        
        return 0
    
    def calculate_steering_angle (self,action:torch.Tensor):
        x = action[:,:,0]
        z = action[:,:,2]

        radius = torch.tensor(x / (z + 1e-9)).to(self.device)

        # k = 1
        # if z*x < 0:
        #     k = -1
        
        l = torch.tensor(0.494).to(self.device) # wheelbase ( front to back )
        w = torch.tensor(0.364).to(self.device) # track (left to right)
        x = torch.sqrt (radius**2 - (l/2)**2)
        phi = torch.atan2 ( radius, l/2)
        return phi

    def calculate_steering_angle_cost (self, action:torch.Tensor): 
         # action shape is ( num_particles, horizon, 3)
        # print (f'shape of action is {action.shape} ')

        steering_angles = self.calculate_steering_angle (action)  # shape: (num_particles, horizon)
        # print (f'shape of steering angles is {steering_angles.shape} average steering angle is {torch.mean(steering_angles).item() * 57.2958} deg ')

        steering_angle_diff = steering_angles[:, 1:] - steering_angles[:, :-1]  # shape: (num_particles, horizon-1)
        # print (f'shape of steering angle diff is {steering_angle_diff.shape} ')

        steering_angle_cost = torch.sum(steering_angle_diff ** 2, dim=1)  # shape: (num_particles)
        # print (f'shape of steering angle cost is {steering_angle_cost.shape} ')

        return steering_angle_cost


    def dynamics(self, s: torch.Tensor, a: torch.Tensor, t=None) -> torch.Tensor:
        """
        Input:
        s: robot global state  (shape: BS x 3)
        a: robot action   (shape: BS x 2)


        Output:
        next robot global state after executing action (shape: BS x 3)
        """
        # print ("s shape is ", s.shape)
        # print ("a shape is ", a.shape)
        assert s.ndim == 2 and s.shape[-1] == 3
        assert a.ndim == 2 and a.shape[-1] == 3

        dt = self.dt        

        # dual ackermann model 
        min_turn_radius = 0.4764
        dx = torch.zeros_like(s)
        L = 0.36

        turning_radius = torch.abs(a[:,0]) / torch.abs(a[:,2] + 1e-6)

        # elementwise mask for samples with small turning radius
        mask = turning_radius < min_turn_radius  # boolean tensor, shape (BS,)

        # compute the curved motion terms
        dx0_curved = a[:,0] * torch.cos(s[:,2]) * torch.cos(a[:,2])
        dx1_curved = a[:,0] * torch.sin(s[:,2]) * torch.cos(a[:,2])
        dx2_curved = 2 * a[:,0] * torch.sin(a[:,2]) / L

        # when turning radius is below threshold, fall back to angular-only update
        dx0_straight = torch.zeros_like(dx0_curved)
        dx1_straight = torch.zeros_like(dx1_curved)
        dx2_straight = a[:,2]

        dx[:,0] = torch.where(mask, dx0_straight, dx0_curved)
        dx[:,1] = torch.where(mask, dx1_straight, dx1_curved)
        dx[:,2] = torch.where(mask, dx2_straight, dx2_curved)

        s3_global = s + dx * dt

        return s3_global





    def terminal_cost(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        
        goal_expanded = self.goal[None, :]
        state_squeezed = state.squeeze()

        # print (f'state_squeezed shape is {state_squeezed.shape}, action shape is {action.shape} ')

        # state_squeezed shaepe is (num_particles, horizon, 3) 
        # action shape is (1, num_particles, horizon, 3)


        # calculated progressive goal cost 
        numerator = torch.norm(goal_expanded-state_squeezed[:,:,:2], dim=2)# (N x T')

        
        denominator = torch.norm (goal_expanded - state_squeezed[:,0,:2], dim=1) + 1e-6  # (N x T')

        # print (f'shape of numerator is {numerator.shape} shape of denominator is {denominator.shape}')

        goal_cost = numerator / denominator.unsqueeze(1)

        goal_cost = goal_cost.mean (dim = 1)

        # calculate terminal goal cost
        terminal_goal_cost = torch.norm(goal_expanded - state_squeezed[:,-1,:2], dim=1)  # (N,)



        # print (f'dimension of goal cost is {goal_cost.shape}')

        # dynamic obstacle cost and social motion cost
        dynamic_obstacle_costs = torch.zeros(self.num_samples).to(self.device)
        sm_costs = torch.zeros(self.num_samples).to(self.device)
    
        
        # action cost -- how sporatic the action is 
        action = action.squeeze()
        # action shape is (num_particles, horizon, 3)
        action_cost = torch.sum((action[:, 1:, 0] - action[:, :-1, 0])**2, dim=1) + torch.sum((action[:, 1:, 2] - action[:, :-1, 2])**2, dim=1)

        # obstacle cost from nav2 costmap 
        pos_in_costmap_frame = (state_squeezed[:,:,:2] - torch.tensor ([self.local_costmap.origin_x, self.local_costmap.origin_y], device=self.device)) 
        grid_x = torch.clamp((pos_in_costmap_frame[:,:,0] / self.local_costmap.resolution).long(), 0, self.local_costmap.width -1)
        grid_y = torch.clamp((pos_in_costmap_frame[:,:,1] / self.local_costmap.resolution).long(), 0, self.local_costmap.height -1)
        costmap_cost = self.local_costmap.data[grid_y.cpu().numpy(), grid_x.cpu().numpy()]  # Shape: (N, T')
        costmap_cost_tensor = torch.tensor(costmap_cost, dtype=torch.float32, device=self.device)
        costmap_cost_tensor = torch.where (costmap_cost_tensor > 90, 10e+8, 0)
        costmap_cost_sum = torch.sum(costmap_cost_tensor, dim=1)
        
        # cv
        cv_cost = torch.zeros(self.num_samples).to(self.device)
        curr_state = self.agent_states.copy()
        prev_state = self.previous_agent_states.copy()
        
        # print (f'shape of state is {state.shape} ')

        # for agent_id in curr_state.keys():
        #     print (f'agent id is {agent_id} ')

        for agent_id in curr_state.keys():
            if agent_id in prev_state.keys():
                
                cv_pred, logits = construct_cv_prediction (curr_state[agent_id], self.agent_velocities[agent_id])
                agent_cost =  compute_cv_cost(state_squeezed, cv_pred)
                cv_cost += agent_cost
                 
                print (f'mean cv cost for agent {agent_id} is {agent_cost.mean()}')

            else:
                cv_pred, logits = construct_cv_prediction (curr_state[agent_id], torch.tensor ([0.0, 0.0], dtype=torch.float32).to(self.device))
                cv_cost += compute_cv_cost (state_squeezed, cv_pred)
                # print (f'id not found in previous state, using current state for cv cost ')
        # print (f'cv cost shape is {cv_cost.shape} ')

        # steering jitter cost 
        steering_cost = self.calculate_steering_angle_cost(action)


        # heading cost 
        heading_to_goal = torch.atan2(self.goal[1] - state_squeezed[:,:,1], self.goal[0] - state_squeezed[:,:,0])  # Shape: (N, T')
        heading_error = heading_to_goal - state_squeezed[:,:,2]  # Shape
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi  # Wrap to [-pi, pi]
        heading_cost = 1 - torch.cos(heading_error)  # Shape: (N, T')
        heading_cost = heading_cost.mean(dim=1)  # Shape: (N,)




        # print (f'average cv cost is {torch.mean(cv_cost).item()} ')
        goal_weight =  1000
        threshold = 0
        action_weight = 1000 # 3000 #20000 #12000
        heading_weight = 1000 # 20 # 200
        dynamic_obstacle_weight = 0
        sm_weight = 10
        costmap_weight = 1
        cv_weight = 100
        terminal_goal_weight = 0 #1000

        print (f'terminal goal cost is {terminal_goal_cost.mean().item() * terminal_goal_weight} \n \
               action cost is {torch.mean(action_cost).item() * action_weight} \n \
               heading cost is  {torch.mean(heading_cost).item() * heading_weight} \n\
               cv cost is {torch.mean(cv_cost).item() * cv_weight} \n \
               mean goal cost is {torch.mean(goal_cost).item() * goal_weight} ')



        return goal_weight*(goal_cost)  + terminal_goal_cost * terminal_goal_weight \
            + action_weight*action_cost \
                + heading_weight * heading_cost  \
                    + dynamic_obstacle_weight*dynamic_obstacle_costs \
                        + sm_weight * sm_costs + cv_weight * cv_cost\
                            + costmap_weight * costmap_cost_sum \
                                + heading_cost * heading_weight

        

    def get_interacting_agents(self):
        self.interacting_agents = []
        robot_state = self.current_state
        for idx, agent_state in self.agent_states.items():
            direction_to_agent = torch.arctan2(agent_state[1] - self.current_state[1], agent_state[0] - self.current_state[0])
            distance_to_agent = torch.norm(agent_state[:2] - self.current_state[:2])

            if self.current_goal_index == 0:
                robot_direction = np.pi/2
            elif self.current_goal_index == 1:
                robot_direction = -np.pi/2

            relative_angle = torch.rad2deg(direction_to_agent -robot_direction)
            relative_angle = (relative_angle + 180) % 360 - 180  
            
            if -90 <= relative_angle <= 90 and distance_to_agent < 2:  
                self.interacting_agents.append(idx)
                self.agent_weights[idx] = (1/distance_to_agent)


    def SocialCost(self, state: torch.Tensor,i,human_states, **kwargs) -> torch.Tensor:
        sm_cost = 0.0
        self.get_interacting_agents()
        # print ("number of interacting agents is ", len(self.interacting_agents))
        if i in self.interacting_agents:
            state_squeezed = state.squeeze()
            r_c = (state_squeezed[:,:,:2] + human_states) / 2
            r_ac = state_squeezed[:,:,:2] - r_c
            r_bc = human_states - r_c
            r_ac_3d = torch.nn.functional.pad(r_ac, (0, 1), "constant", 0)  # [N, T', 3]
            r_bc_3d = torch.nn.functional.pad(r_bc, (0, 1), "constant", 0)  # [N, T', 3]
            robot_velocity_3d = torch.nn.functional.pad(self.robot_velocity, (0, 1), "constant", 0)  # Shape: [3]
            agent_velocities_3d = torch.nn.functional.pad(self.agent_velocities[i], (0, 1), "constant", 0) 
            l_ab = torch.cross(r_ac_3d, robot_velocity_3d[None,None,:], dim=2) + torch.cross(r_bc_3d, agent_velocities_3d[None,None,:], dim=2)
            l_ab = l_ab[:, :, 2]
            l_ab_dot_product = l_ab[:, :-1] * l_ab[:, 1:]    # Determine if dot product is positive or not
            condition = l_ab_dot_product > 0  # Shape: [N, T'-1]
            l_ab_conditional = torch.where(condition, -11*torch.abs(l_ab[:, :-1]), torch.tensor(10.0, device=l_ab.device))  # Shape: [N, T'-1]
            sm_cost += torch.sum(l_ab_conditional, dim=1)
        return sm_cost
        
    def collision_avoidance_cost(self,state):
        xy_coords = state[..., :2].cpu().numpy()  # Shape: (N, T', 2)
        flattened_coords = xy_coords.reshape(-1, 2)
        x_min, y_min, x_max, y_max = self.bounds
        within_bounds = (
            (flattened_coords[:, 0] >= x_min) &
            (flattened_coords[:, 0] <= x_max) &
            (flattened_coords[:, 1] >= y_min) &
            (flattened_coords[:, 1] <= y_max)
        )

        # Use Shapely's vectorized `contains` for points within bounds
        collision_flags = contains(self.multi_polygon, flattened_coords[:, 0], flattened_coords[:, 1])
        collision_flags[~within_bounds] = False  # Points outside bounds are not collisions

        # Assign costs based on collision flags
        costs = torch.where(
            torch.tensor(collision_flags, dtype=torch.bool),
            torch.tensor(10.0),  
            torch.tensor(0.0) 
        )
        costs = costs.view(state.shape[0], state.shape[1]).sum(dim=1)

        return costs.to(state.device)


