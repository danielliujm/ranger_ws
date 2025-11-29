import torch
from pytorch_mppi import MPPI, KMPPI, mppi
from alpaca_navigation.config import *
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.vectorized import contains
from nav_msgs.msg import OccupancyGrid
import math

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
        cov[1, 1] = 0.001 #0.001
        cov[2, 2] = 0.5 #0.001
        # MPPI initialization
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
            # lambda_ = 100,
            kernel = mppi.RBFKernel (sigma = 2.5),
            num_support_pts=2,
            # w_action_seq_cost=0.0,
            u_min=torch.tensor([0.0, -0.0, -1.5], dtype=torch.float32).to(self.device),
            u_max=torch.tensor([0.4, 0.0, 1.5], dtype=torch.float32).to(self.device),
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
        rollouts = self.mppi.get_rollouts (current_state, num_rollouts = NUM_SAMPLES)
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

        # self.s2_ego.zero_()
        # s2_ego = torch.zeros_like(s).to(self.device)
        # s2_ego = self.s2_ego
        # d_theta = a[:, 1] * dt
        # turning_radius =  a[:, 0] / a[:, 1]


        # vx,vy, omega = a[:,0], a[:,1], a[:,2]        
        # theta = s[:,2]

   
        
        # dx = vx* torch.cos(theta) - vy*torch.sin(theta)
        # dy = vx* torch.sin(theta) + vy*torch.cos(theta)
        # dtheta = omega * dt
        
        # s2_global = torch.zeros_like(s)
    
        
        # s2_global [:,0] = s[:,0] + dx*dt
        # s2_global [:,1] = s[:,1] + dy*dt
        # s2_global [:,2] = normalize_angle(s[:,2] +  dtheta)
        
        
        
        # heading = torch.atan2(a[:,1], a[:,0])  
        # velocity = torch.sqrt (a[:,0]**2 + a[:,1]**2) #*torch.sign (a[:,0])

    

        # # heading [torch.abs(heading > np.pi/2)] = torch.sign (heading [torch.abs(heading > np.pi/2)]) * np.pi/2

        # s3_global = torch.zeros_like(s)
        # dx = velocity* torch.cos (s[:,2] + heading )
        # dy = velocity* torch.sin (s[:,2] + heading )

        # s3_global [:,0] = s[:,0] + dx*dt
        # s3_global [:,1] = s[:,1] + dy*dt
        # s3_global [:,2] = s[:,2] 

        
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

        # state_squeezed shaepe is (num_particles, horizon, 2) 
     

        # calculated goal cost 
        dist = torch.norm(goal_expanded-state_squeezed[:,:,:2], dim=2)# (N x T')
        goal_cost = torch.sum(dist, dim=1)
        dynamic_obstacle_costs = torch.zeros(self.num_samples).to(self.device)
        sm_costs = torch.zeros(self.num_samples).to(self.device)
    
        
        # action cost
        action = action.squeeze()
        action_cost = torch.sum((action[:, 1:, 1] - action[:, :-1, 1])**2, dim=1)

        # obstacle cost from nav2 costmap 
        pos_in_costmap_frame = (state_squeezed[:,:,:2] - torch.tensor ([self.local_costmap.origin_x, self.local_costmap.origin_y], device=self.device)) 
        grid_x = torch.clamp((pos_in_costmap_frame[:,:,0] / self.local_costmap.resolution).long(), 0, self.local_costmap.width -1)
        grid_y = torch.clamp((pos_in_costmap_frame[:,:,1] / self.local_costmap.resolution).long(), 0, self.local_costmap.height -1)
        costmap_cost = self.local_costmap.data[grid_y.cpu().numpy(), grid_x.cpu().numpy()]  # Shape: (N, T')
        costmap_cost_tensor = torch.tensor(costmap_cost, dtype=torch.float32, device=self.device)
        costmap_cost_tensor = torch.where (costmap_cost_tensor > 90, 10e+8, 0)
        costmap_cost_sum = torch.sum(costmap_cost_tensor, dim=1)



        # heading cost 
        heading_cost = torch.sum((torch.atan2(action[:, 1:, 1], action[:, 1:, 0]) - torch.atan2(action[:, :-1, 1], action[:, :-1, 0]))**2, dim=1)

        for i in range(ACTIVE_AGENTS):
            next_x_states = self.agent_states[i][0] + torch.linspace(self.dt,self.horizon*self.dt,self.horizon,device=self.device)* self.agent_velocities[i][0]
            next_y_states = self.agent_states[i][1] + torch.linspace(self.dt,self.horizon*self.dt,self.horizon,device=self.device)* self.agent_velocities[i][1]
            human_states  = torch.stack((next_x_states,next_y_states), dim=1)
            dist = torch.norm(state_squeezed[:,:,:2] -  human_states,dim=2)
            # social mometum cost
            sm_costs += self.SocialCost(state, i, human_states)
            #dynamic obstacle cost
            
            dynamic_obstacle_cost = torch.where(dist < 1.5, 1/(0.1+dist**2), torch.tensor(0.0, device=self.device))
            dynamic_obstacle_costs += torch.sum(dynamic_obstacle_cost,dim=1)
        
        # print ("goal_cost shape is ", goal_cost.shape)
        # print ("action_cost shape is ", action_cost.shape)

        goal_weight =  110 
        threshold = 0
        action_weight = 20000 #12000
        heading_weight = 0
        if torch.sum (dynamic_obstacle_cost) > threshold:
            heading_weight = 1000#1000
            
        else: 
            heading_weight = 0#6000
        dynamic_obstacle_weight = 200
        sm_weight = 10
        costmap_weight = 1


        return goal_weight*(goal_cost) + action_weight*action_cost + heading_weight * heading_cost  + dynamic_obstacle_weight*dynamic_obstacle_costs + sm_weight * sm_costs + costmap_weight * costmap_cost_sum #+ 10*yaw_cost# + 5*static_costs  #weights can be tuned for desired behaviour
    

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


