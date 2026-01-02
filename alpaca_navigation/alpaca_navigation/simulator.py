from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import FancyArrowPatch
import numpy as np
import torch

from mppi_config import *

import torch
from pytorch_mppi import MPPI, KMPPI, mppi
from mppi_config import *
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.vectorized import contains
from nav_msgs.msg import OccupancyGrid
import math

# from alpaca_navigation.cv import *


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
        cov[0, 0] = 1
        cov[1, 1] = 10e-8 #0.001
        cov[2, 2] = 0.002
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


            u_min=torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32).to(self.device),
            u_max=torch.tensor([0.6, 0.0, 1.0], dtype=torch.float32).to(self.device),
            
            # action_max=torch.tensor([0.6, 0.0, 1.0], dtype=torch.float32).to(self.device),
            # action_min=torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32).to(self.device),

            lambda_ = 1, #1e-2,
            # delta_t = 0.1,
            kernel = mppi.RBFKernel (sigma = 3.0),
            num_support_pts=self.horizon //4, 

            noise_abs_cost = False, 


            # w_action_seq_cost=1000,
            u_per_command = HORIZON_LENGTH,

           

            # U_init = self.debug_init_action
        )

        self.candidate_states = None
        self.candidate_costs = None

        self.prev_u = None
    
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
        action = self.mppi.command(current_state, shift_nominal_trajectory= True)
        self.mppi.u_init = action if action.dim() ==1 else action[0]

    

        rollouts = self.mppi.get_rollouts (current_state, num_rollouts = 1)
        costs = self.mppi.cost_total.squeeze(0)

        termination =  torch.linalg.norm(self.current_state[:2] - self.goal) < TERMINATION_TOLERANCE
        

        self.prev_u = action 

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
    

    def steering_angle_single_action (self, action:torch.Tensor):
        x = action[0]
        z = action[2]

        radius = x / (z + 1e-9)

        is_ackermann = True if abs(radius >= MIN_TURN_RADIUS) else False
        # k = 1
        # if z*x < 0:
        #     k = -1
        
        l = 0.494  # wheelbase ( front to back )
        w = 0.364  # track (left to right)

        print (f'calculated turning radius is {radius} m ')

        # x = math.sqrt (radius**2 - (l/2)**2)
        phi = math.atan2 ( l/2, radius)
        return phi, is_ackermann
    
    def calculate_steering_angle (self,action:torch.Tensor):
        x = action[:,:,0]
        z = action[:,:,2]

        radius = torch.tensor(x / (z + 1e-9)).to(self.device)
        is_ackermann = torch.abs(radius) >= MIN_TURN_RADIUS

        # k = 1
        # if z*x < 0:
        #     k = -1
        
        l = torch.tensor(0.494).to(self.device) # wheelbase ( front to back )
        w = torch.tensor(0.364).to(self.device) # track (left to right)
        x = torch.sqrt (radius**2 - (l/2)**2)
        phi = torch.atan2 ( l/2, radius)
        return phi, is_ackermann

    


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
        L = 0.494  # wheelbase

        turning_radius = torch.abs(a[:,0]) / torch.abs(a[:,2] + 1e-6)

        # elementwise mask for samples with small turning radius
        mask = turning_radius < min_turn_radius  # boolean tensor, shape (BS,)

        # compute the curved motion terms
        dx0_curved = a[:,0] * torch.cos(s[:,2]) * torch.cos(a[:,2])
        dx1_curved = a[:,0] * torch.sin(s[:,2]) * torch.cos(a[:,2])
        dx2_curved = 2 * a[:,0] * torch.sin(a[:,2]) / L

        # when turning radius is below threshold, fall back to angular-only update
        dx0_rotate = torch.zeros_like(dx0_curved)
        dx1_rotate = torch.zeros_like(dx1_curved)
        dx2_rotate = a[:,2]

        dx[:,0] = torch.where(mask, dx0_rotate, dx0_curved)
        dx[:,1] = torch.where(mask, dx1_rotate, dx1_curved)
        dx[:,2] = torch.where(mask, dx2_rotate, dx2_curved)
        s3_global = s + dx * dt


        s3_global[:,2] = torch.remainder (s3_global[:,2], 2.0 * np.pi)

        return s3_global

    def goal_progress_cost (self, state: torch.Tensor) -> torch.Tensor:
        goal_expanded = self.goal[None, :]
        state_squeezed = state.squeeze()

        numerator = torch.norm (goal_expanded - state_squeezed [:,1:,:2], dim = 2)
        denominator = torch.norm (goal_expanded - state_squeezed [:,:-1,:2], dim = 2) + 1e-6

        # print (f'numerator shape is {numerator.shape} , denominator shape is {denominator.shape} ')
        cost = numerator / denominator 
        
        cost = cost.mean (dim = 1)

        return cost 
    
    def terminal_goal_cost (self, state: torch.Tensor) -> torch.Tensor:
        goal_expanded = self.goal [None,:]
        state_squeezed = state.squeeze()
        cost = torch.norm(goal_expanded - state_squeezed[:,-1,:2], dim=1)  # (N,)
        return cost

    
    
    def steering_cost_i_axis (self, action: torch.Tensor) -> torch.Tensor: 
        if self.prev_u is not None:
            action = action.squeeze()  # shape is (num_particles, horizon, 3)

            # print (f'shape of prev steering angle is {self.prev_u[0,:].shape} ')
            prev_steering_angle, prev_is_ackermann = self.steering_angle_single_action (self.prev_u[0,:])
            
            curr_steering_angle, curr_is_ackermann  = self.calculate_steering_angle (action)

            # print (f'shape of curr steering angle is {curr_steering_angle.shape} ')

            steering_angle_diff = curr_steering_angle[:,0] - prev_steering_angle

            mode_switch = curr_is_ackermann[:,0].float() != prev_is_ackermann
            mode_switch_penalty = mode_switch * MS_COST
            # print (f'shape of mode switch penalty is {mode_switch_penalty} ')
            action_cost =  steering_angle_diff ** 2  #+ mode_switch_penalty.float()   # 
                                                                           
        else: 
            action_cost = torch.zeros (self.num_samples).to(self.device)
        return action_cost
    
    def steering_cost_t_axis (self, action:torch.Tensor): 
         # action shape is ( num_particles, horizon, 3)
        # print (f'shape of action is {action.shape} ')

        steering_angles = self.calculate_steering_angle (action)  # shape: (num_particles, horizon)
        # print (f'shape of steering angles is {steering_angles.shape} average steering angle is {torch.mean(steering_angles).item() * 57.2958} deg ')

        steering_angle_diff = steering_angles[:, 1:] - steering_angles[:, :-1]  # shape: (num_particles, horizon-1)
        # print (f'shape of steering angle diff is {steering_angle_diff.shape} ')

        steering_angle_cost = torch.sum(steering_angle_diff ** 2, dim=1)  # shape: (num_particles)
        # print (f'shape of steering angle cost is {steering_angle_cost.shape} ')

        return steering_angle_cost
    
    def action_cost_i_axis (self, action: torch.Tensor) -> torch.Tensor:

        if self.prev_u is not None:
            action = action.squeeze()  # shape is (num_particles, horizon, 3)
            acceleration = action [:,:,0] - self.prev_u[None,:,0]
            # print (f'shape of acceleration is {acceleration.shape} ')
            action_cost = torch.sum (acceleration **2 ,  dim = 1)
            # print (f'shape of action_cost is {action_cost.shape} ')
            
                                                                     
        
        else: 
            action_cost = torch.zeros (self.num_samples).to(self.device) 
        return action_cost
    
    def action_cost_t_axis (self, action: torch.Tensor) -> torch.Tensor:
        action = action.squeeze()  # shape is (num_particles, horizon, 3)
        action_diff = action[:,1:,:] - action[:,:-1,:]
        action_cost = torch.sum (action_diff **2 ,  dim = (1,2))
        return action_cost
    
    def costmap_cost (self, state: torch.Tensor) -> torch.Tensor:
        state_squeezed = state.squeeze()  #  (num_particles, horizon, 3)
        pos_in_costmap_frame = (state_squeezed[:,:,:2] - torch.tensor ([self.local_costmap.origin_x, self.local_costmap.origin_y], device=self.device)) 
        grid_x = torch.clamp((pos_in_costmap_frame[:,:,0] / self.local_costmap.resolution).long(), 0, self.local_costmap.width -1)
        grid_y = torch.clamp((pos_in_costmap_frame[:,:,1] / self.local_costmap.resolution).long(), 0, self.local_costmap.height -1)
        costmap_cost = self.local_costmap.data[grid_y.cpu().numpy(), grid_x.cpu().numpy()]  # Shape: (N, T')

        costmap_cost_tensor = torch.tensor(costmap_cost, dtype=torch.float32, device=self.device)
        
        costmap_cost_tensor = torch.where (costmap_cost_tensor > 90, 10e+8, costmap_cost_tensor)
        costmap_cost_tensor = costmap_cost_tensor
        costmap_cost_sum = torch.sum(costmap_cost_tensor, dim=1)

        return costmap_cost_sum

    def cv_cost (self, state: torch.Tensor) -> torch.Tensor:
        state_squeezed = state.squeeze()  #  (num_particles, horizon, 3)
        cv_cost = torch.zeros(self.num_samples).to(self.device)
        curr_state = self.agent_states.copy()
        prev_state = self.previous_agent_states.copy()

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
        return cv_cost
    
    def heading_cost (self, state: torch.Tensor) -> torch.Tensor:
        state_squeezed = state.squeeze()  #  (num_particles, horizon, 3)
        heading_to_goal = torch.atan2(self.goal[1] - state_squeezed[:,:,1], self.goal[0] - state_squeezed[:,:,0])  # Shape: (N, T')
        heading_error = heading_to_goal - state_squeezed[:,:,2]  # Shape
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi  # Wrap to [-pi, pi]
        heading_cost = 1 - torch.cos(heading_error)  # Shape: (N, T')
        heading_cost = heading_cost.mean(dim=1)  # Shape: (N,)

        return heading_cost

    def terminal_cost(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        
        goal_expanded = self.goal[None, :] # (num_particles, horizon, 3) 
        state_squeezed = state.squeeze() #  (num_particles, horizon, 3)

         # calculate terminal goal cost
        terminal_goal_cost = self.terminal_goal_cost(state)

        goal_cost = self.goal_progress_cost (state)

        action_cost = self.action_cost_i_axis(action) 

        action_cost_t = self.action_cost_t_axis(action)

        costmap_cost = torch.zeros (self.num_samples, device = self.device)
        
        cv_cost = self.cv_cost(state)
        
        steering_cost = self.steering_cost_i_axis(action)

        heading_cost = self.heading_cost(state)


        goal_weight =  5000 
        action_weight = 1000
        heading_weight = 1000 
        steering_weight = 1000

        sm_weight = 10
        costmap_weight = 1 
        cv_weight = 100
        terminal_goal_weight = 1000
        action_t_weight = 1000

        print (f'min terminal goal cost is {terminal_goal_cost.min().item() * terminal_goal_weight} \n \
               mean action cost is {torch.mean(action_cost).item() * action_weight} min, max are {action_cost.min().item() * action_weight}, {action_cost.max().item() * action_weight} \n \
               mean action_t cost is {torch.mean(action_cost_t).item() * action_t_weight} \n \
               mean heading cost is  {torch.mean(heading_cost).item() * heading_weight} \n\
               mean steering cost is {torch.mean(steering_cost).item() * steering_weight} min, max are {steering_cost.min().item() * steering_weight}, {steering_cost.max().item() * steering_weight} \n \
                min costmap cost is {torch.min(costmap_cost).item() * costmap_weight} \n \
               min cv cost is {torch.min(cv_cost).item() * cv_weight} \n \
               mean goal cost is {torch.mean(goal_cost).item() * goal_weight} min, max are {goal_cost.min().item() * goal_weight}, {goal_cost.max().item() * goal_weight} ')


        cost = goal_weight*(goal_cost)  + terminal_goal_cost * terminal_goal_weight \
            + action_weight*action_cost   + action_t_weight * action_cost_t \
                + steering_weight * steering_cost \
                    + heading_weight * heading_cost  \
                            + cv_weight * cv_cost\
                                + costmap_weight * costmap_cost
        
        ###### for visualization ######
        self.candidate_states = state_squeezed
        self.candidate_costs = cost
        ############################## 


        return  cost

    def get_candidate_states_and_costs(self):
        return self.candidate_states, self.candidate_costs
        

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





def my_dynamics(s: torch.Tensor, a: torch.Tensor, t=None) -> torch.Tensor:
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

        dt = HZ 

        # dual ackermann model 
        min_turn_radius = 0.4764
        dx = torch.zeros_like(s)
        L = 0.494  # wheelbase

        turning_radius = torch.abs(a[:,0]) / torch.abs(a[:,2] + 1e-6)

        # elementwise mask for samples with small turning radius
        mask = turning_radius < min_turn_radius  # boolean tensor, shape (BS,)

        # compute the curved motion terms
        dx0_curved = a[:,0] * torch.cos(s[:,2]) * torch.cos(a[:,2])
        dx1_curved = a[:,0] * torch.sin(s[:,2]) * torch.cos(a[:,2])
        dx2_curved = 2 * a[:,0] * torch.sin(a[:,2]) / L

        # when turning radius is below threshold, fall back to angular-only update
        dx0_rotate = torch.zeros_like(dx0_curved)
        dx1_rotate = torch.zeros_like(dx1_curved)
        dx2_rotate = a[:,2]

        dx[:,0] = torch.where(mask, dx0_rotate, dx0_curved)
        dx[:,1] = torch.where(mask, dx1_rotate, dx1_curved)
        dx[:,2] = torch.where(mask, dx2_rotate, dx2_curved)
        s3_global = s + dx * dt

        return s3_global


def main():
    time_total = 0
    dt = HZ
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    loc = torch.tensor ([0.0, 0.0, 0.0], device=device)
    prev_loc = torch.tensor ([0.0, 0.0, 0.0], device=device)
    vel = torch.tensor ([0.0, 0.0, 0.0], device=device)
    goal = torch.tensor ([10.0, 0.0])

    mppi = SMMPPIController (STATIC_OBSTACLES, device=device)
    mppi.set_goal (goal)

    termination = False
    loc_history = []  # Track positions for visualization

    x_history = []
    z_history = []


    while not termination and time_total < 30: 
        action,_, _, termination = mppi.compute_control(loc, prev_loc, vel, {}, {}, {})


        # action = torch.mean (action[None, :10,:], dim = 1)
        x_history.append (action[0,0].detach().cpu().numpy())
        z_history.append (action[0,2].detach().cpu().numpy())

        
        prev_loc = loc


        loc = my_dynamics (loc[None,:], action[None,:,:]).squeeze(0)
        loc [2] = torch.remainder (loc[2], 2.0 * np.pi)

        
        loc_history.append(loc.detach().clone().cpu())
        time_total += dt


        print (f'elapsed time {time_total}, z command is {action[0,2].item()}')

    if loc_history:
        loc_hist = torch.stack(loc_history).numpy()
        xs, ys = loc_hist[:, 0], loc_hist[:, 1]
        yaws = loc_hist[:, 2] if loc_hist.shape[1] > 2 else np.zeros_like(xs)

        fig, ax = plt.subplots()
        x_min, x_max = xs.min(), xs.max()
        y_min, y_max = ys.min(), ys.max()
        pad_ratio = 0.1
        x_pad = max((x_max - x_min) * pad_ratio, 1e-2)
        y_pad = max((y_max - y_min) * pad_ratio, 1e-2)
        ax.set_xlim(0,10)
        ax.set_ylim(-10,10)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("XY Trajectory")
        ax.grid(True)

        line, = ax.plot([], [], "b-", lw=2)
        point, = ax.plot([], [], "ro")
        arrow = FancyArrowPatch((0, 0), (0, 0), color="orange", mutation_scale=12, linewidth=1.5)
        arrow.set_visible(False)
        ax.add_patch(arrow)
        arrow_length = 1

        def init():
            line.set_data([], [])
            point.set_data([], [])
            arrow.set_visible(False)
            return line, point, arrow

        def update(frame_idx):
            line.set_data(xs[: frame_idx + 1], ys[: frame_idx + 1])
            point.set_data([xs[frame_idx]], [ys[frame_idx]])
            heading = yaws[frame_idx]
            dx = arrow_length * np.cos(heading)
            dy = arrow_length * np.sin(heading)
            start_x, start_y = xs[frame_idx], ys[frame_idx]
            arrow.set_positions((start_x, start_y), (start_x + dx, start_y + dy))
            arrow.set_visible(True)
            return line, point, arrow

        ani = animation.FuncAnimation(
            fig,
            update,
            init_func=init,
            frames=len(xs),
            interval=50,
            blit=True,
        )

        gif_path = Path(__file__).with_name("xy_trajectory.gif")
        ani.save(gif_path, writer=animation.PillowWriter(fps=20))
        plt.close(fig)
        print(f"Saved XY trajectory animation to {gif_path}")

    
    # loc_hist = torch.stack(loc_history).numpy()
    plt.plot (z_history, label = 'yaw rate command')
    # plt.savefig (Path(__file__).with_name ('simulator_yaw_loc_history.png'))
    
    plt.plot (x_history, label = 'forward command')
    plt.legend()
    plt.savefig (Path(__file__).with_name ('simulator_action_history.png'))
if __name__ == "__main__":
    main()