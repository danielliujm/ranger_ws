import math

import torch

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class VisualizationUtils:
    def __init__(self, node: Node) -> None:
        self._node = node

        self._rollouts_pub = self._node.create_publisher(
            MarkerArray, f"/{self._node.get_name()}/vis/rollouts", 1
        )

        self._path_pub = self._node.create_publisher(
            Path, f"/{self._node.get_name()}/vis/path", 1
        )

    def visualize_rollouts(self, rollouts: torch.Tensor, costs: torch.Tensor) -> None:
        """
        Input:
        rollouts: (shape: 1 x NUM_SAMPLES x HORIZON x 3)
        costs: (shape: NUM_SAMPLES)
        """

        rollouts = rollouts.unsqueeze(0)
        var = torch.var (costs, dim = 0 )
        # print ("cost variance is ", var.item())

        # print (f'visualizing rollouts with shape {rollouts.shape} and costs shape {costs.shape}')

        assert rollouts.ndim == 4 and rollouts.shape[0] == 1 and rollouts.shape[-1] == 3

        # print ("passed assert")

        min_cost = torch.min(costs).item()
        max_cost = torch.max(costs).item()

        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        


        # min cost trajectory 
        min_cost_id = torch.argmin (costs).item()
        max_cost_id = torch.argmax (costs).item()




        rollouts = rollouts.cpu().numpy()

        ###################################################
        # ro = rollouts [0,:,:,0] 
        # np.save ('cost.npy', costs.cpu().numpy())


        # rollout_1 = rollouts[:,min_cost_id,:,:]
        # diff = rollouts - np.expand_dims (rollout_1, axis = 1)
        # print (f'diff shape: {diff.shape}')

        # dist = np.linalg.norm (diff, axis=3)

        # dist_sum = np.sum (dist, axis = 2)

        # print (f'dist shape: {dist.shape}, dist_sum shape: {dist_sum.shape}')

        # mean_dist = np.mean (dist_sum, axis = 1)


        # print (f'rollout mean dist from min cost rollout: {mean_dist[0]}')
        ###################################################

        
        min_marker = Marker()
        min_marker.header.frame_id = "map"
        min_marker.id = min_cost_id
        min_marker.type = Marker.LINE_STRIP
        min_marker.scale.x = 0.005
        min_marker.scale.y = 0.005
        min_marker.scale.z = 0.005
        min_marker.lifetime = Duration(seconds= 0.07).to_msg()  
        min_marker.color.g = min_marker.color.a = 1.0
        
        for state_idx in range(rollouts.shape[2]):
            state = rollouts [0,0, state_idx,:]
            min_marker.points.append (Point(x=state[0].item(), y=state[1].item()))
        
        # max_marker = Marker()
        # max_marker.header.frame_id = "odom"
        # max_marker.id = max_cost_id
        # max_marker.type = Marker.LINE_STRIP
        # max_marker.scale.x = 0.005
        # max_marker.scale.y = 0.005
        # max_marker.scale.z = 0.005
        # max_marker.lifetime = Duration(seconds=0.1).to_msg()  
        # max_marker.color.r = max_marker.color.a = 1.0
        
        # for state_idx in range (rollouts.shape[2]):
        #     state = rollouts [0,0, state_idx,:]
        #     max_marker.points.append (Point(x=state[0].item(), y=state[1].item()))

        
        marker_array.markers.append (min_marker)


        # max cost trajectory
        # marker_array.markers.append (max_marker)
    
        
        # marker = Marker()
        # for sample_idx in range(rollouts.shape[1]):
        #     marker = Marker()
        #     marker.header.frame_id = "odom"
        #     marker.id = sample_idx
        #     marker.type = Marker.SPHERE_LIST
        #     marker.scale.x = 0.005
        #     marker.scale.y = 0.005
        #     marker.scale.z = 0.005
        #     marker.lifetime = Duration(seconds=1.0).to_msg()  # type: ignore
            

        #     cost = costs[sample_idx].item()
        #     if cost == min_cost:
        #         marker.color.r = marker.color.g = marker.color.a = 1.0
        #     else:
        #         cost_prop = (cost - min_cost) / (max_cost - min_cost)
        #         # Smooth transition from red -> yellow -> green
        #         # prop: 1.0 -> 0.5 -> 0.0
        #         # r   : 1.0 -> 1.0 -> 0.0
        #         # g   : 0.0 -> 1.0 -> 1.0
        #         marker.color.r = 1.0 #min(1.0, 2 * cost_prop)
        #         marker.color.g = 0.0 #max(0.0, 1 - 2 * cost_prop)
        #         marker.color.a = 1.0
                
                
            # min_cost_id = torch.argmin (costs)
            # print ("min cost id is ", min_cost_id)

            # for state_idx in range(rollouts.shape[2]):
            #     state = rollouts[0, min_cost_id, state_idx,:]
            #     marker.points.append(Point(x=state[0].item(), y=state[1].item()))
                
            
            # max_cost_id = torch.argmax (costs)
            # for state_idx in range(rollouts.shape[2]):
            #     state = rollouts [0, max_cost_id, state_idx, :]
            #     marker.points.append(Point(x=state[0].item(), y=state[1].item()))
            
            # marker_array.markers.append (marker)
            


        self._rollouts_pub.publish(marker_array)

    def visualize_path(self, path: list[tuple[float, float, float]]) -> None:
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self._node.get_clock().now().to_msg()

        for state in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self._node.get_clock().now().to_msg()

            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation.w = math.cos(state[2] / 2)
            pose.pose.orientation.z = math.sin(state[2] / 2)

            path_msg.poses.append(pose)

        self._path_pub.publish(path_msg)
