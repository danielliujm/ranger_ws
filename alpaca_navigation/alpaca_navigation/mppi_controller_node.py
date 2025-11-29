import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import torch
from pytorch_mppi import MPPI
from alpaca_navigation.mppi_config import *
from alpaca_navigation.sm_mppi import SMMPPIController
# from utils import dynamics, normalize_angle, save_data
import numpy as np
import math
import time
from shapely.geometry import Polygon, MultiPolygon, Point
from shapely.vectorized import contains
from alpaca_navigation.sm_mppi import SMMPPIController
from datetime import datetime
# from vis_utils import *
from visualization_msgs.msg import Marker, MarkerArray
# from ranger_msgs.srv import RotateInPlace
from rclpy.executors import MultiThreadedExecutor
from playsound import playsound
import pyttsx3
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped;
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from vision_msgs.msg import Detection3D, Detection3DArray



EPSILON = 1e-12

class MPPLocalPlannerMPPI(Node):
    def __init__(self):
        super().__init__('mpc_local_planner_mppi')

        # Initialize parameters

        
        self.rollouts = torch.zeros((7, NUM_SAMPLES, 2))
        self.costs = torch.zeros((7, NUM_SAMPLES, 2))
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


        # ROS2 setup
        self.cbgroup = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.timer = self.create_timer(HZ, self.plan_and_publish, callback_group=self.cbgroup)
        self.time_steps = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.start_time = time.time()
        self.controller = SMMPPIController(STATIC_OBSTACLES, self.device)
        self.counter = 0
        self.filtered_action = torch.tensor([0.0, 0.0,0.0], dtype=torch.float32).to(self.device)
        self.alpha = 0.8
        

        self.current_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)  # [x, y, yaw]
        self.robot_velocity = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device)
        self.previous_robot_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)

        self.agent_states = {i: torch.tensor([10.0, 10.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.agent_velocities = {i: torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        self.previous_agent_states = {i: torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}

        self.prev_control = 0.0
        self.control_variation = 0.0
        self.speak = False
        self.last_speak_time = 0.0

        self.current_robot_pose = None

        # subscribe to current position 
        self.robot_pose_subscriber = self.create_subscription( PoseWithCovarianceStamped, '/amcl_pose', self.robot_pose_cb, 10)

        # `PlanSegmenter` publishes the current goal as a `PoseStamped` on `/goal`.
        # Subscribe to `/goal` to receive the published goals.
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)
        self.curr_goal = None

        # Create a client for the `/next_goal` Trigger service offered by `plan_segmenter`.

        self.next_goal_client = self.create_client(Trigger, '/next_goal')
        if not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            # Service may come up later; log and continue. We'll handle unavailable service when calling.
            self.get_logger().info('`/next_goal` service not available yet. Will try when needed.')

        
        # subscribe to local costmap 
        self.local_costmap_sub = self.create_subscription (OccupancyGrid, '/local_costmap/costmap', self.local_costmap_cb, 10)

        # subscribe to human detection results 
        self.detection_subscriber = self.create_subscription (Detection3DArray, '/SimpleTrack_results', self.human_detection_cb, 10)

        # publish cost to debug
        self.cost_pub = self.create_publisher (Float32, '/mppi_controller/cost', 10)
        


    def goal_cb (self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = 2.0 * math.atan2(
            msg.pose.orientation.z, msg.pose.orientation.w
        )  # NOTE: assuming roll and pitch are negligible

        self.curr_goal = torch.tensor ([x, y], dtype=torch.float32).to(self.controller.device)

        
        self.controller.set_goal(self.curr_goal)
        self.get_logger().info(f'new goal received: {self.curr_goal}')
    
    def human_detection_cb (self, msg):

    def robot_pose_cb (self, msg):
        self.get_logger().debug('found robot pose')
        self.current_robot_pose = msg

    def local_costmap_cb (self, msg):
        self.get_logger().info ('received local costmap')
        self.controller.set_local_costmap (msg)



    def plan_and_publish(self):
        
        if self.curr_goal is None:
            self.get_logger().info('waiting for goal ...')
            return
        self.get_logger().info(f'the goal is {self.curr_goal}, the controller running at HZ {HZ}')

        self.counter += 1


        if self.current_robot_pose is None:
            self.get_logger().warn("Can't find robot pose")
            return
            # Convert pose to MPPI state representation
        yaw = 2.0 * math.atan2(
            self.current_robot_pose.pose.pose.orientation.z, self.current_robot_pose.pose.pose.orientation.w
        )  # NOTE: assuming roll and pitch are negligible
        
        
        self.current_state = torch.tensor(
                [
                    self.current_robot_pose.pose.pose.position.x,
                    self.current_robot_pose.pose.pose.position.y,
                    yaw,
                ],
                dtype=torch.float32,
            ).to(self.device)
        
        #############################################################################
        #############################################################################
        
        if self.counter == 3:
            self.previous_robot_state = self.current_state
            
        if torch.any(self.previous_robot_state):
            self.robot_velocity = (self.current_state[:2] - self.previous_robot_state[:2])/HZ
            self.previous_robot_state = self.current_state


        action, self.rollouts, self.costs, termination = self.controller.compute_control(
            self.current_state, self.previous_robot_state, self.robot_velocity, self.agent_states, self.previous_agent_states, self.agent_velocities
        )

        self.cost_pub.publish (Float32(data = torch.min (self.costs).item()))
        
        self.rollouts = self.rollouts.unsqueeze(0)


        if action is not None and not termination:
            # print ("visualization took ", (datetime.now().microsecond-b4_time.microsecond)/1000, "ms")

            twist_stamped = Twist()
            # twist_stamped.header.stamp = self.get_clock().now().to_msg()
            
            self.filtered_action = self.alpha * action + (1 - self.alpha) * self.filtered_action
            action = self.filtered_action        
            
            x_effort= action[0].item() if abs(action[0].item()) < VMAX else np.sign(action[0].item())*VMAX 
            y_effort = action[1].item() if abs(action[1].item()) < VMAX else np.sign(action[1].item())*VMAX 

            if abs(x_effort) < 0.03:
                x_effort = 0.0
            if abs (y_effort) < 0.03:
                y_effort = 0.0
            if abs (action[2].item()) < 0.1:
                action[2] = 0.0


            self.get_logger().info(f'x y z {x_effort} {y_effort} {action[2].item()}')

            
            twist_stamped.linear.x = x_effort 
            twist_stamped.linear.y = y_effort
            twist_stamped.angular.z = action[2].item()

            self.control_variation += np.sqrt((self.prev_control - y_effort)**2)
            self.prev_control = y_effort
           
            # twist_stamped.linear.x = 0.0
            # twist_stamped.linear.y = 0.0
            # twist_stamped.angular.z = 0.0

            # twist_stamped.angular.z = action[1].item() #min(action[1].item(), VMAX)
            self.cmd_vel_pub.publish(twist_stamped)
                      
            self.linear_velocities.append(x_effort)
            self.angular_velocities.append(y_effort)
            self.time_steps.append(time.time() - self.start_time)

           
        elif termination:
            self.get_logger().info('Reached the goal!!!!!')
            # Call the PlanSegmenter's `/next_goal` service to request publishing the next goal.
            try:
                if self.next_goal_client.service_is_ready():
                    req = Trigger.Request()
                    future = self.next_goal_client.call_async(req)

                    def _on_next_goal_done(fut):
                        try:
                            res = fut.result()
                            if res.success:
                                self.get_logger().info(f"/next_goal: {res.message}")
                            else:
                                self.get_logger().info(f"/next_goal (not success): {res.message}")
                        except Exception as e:
                            self.get_logger().error(f"Failed calling /next_goal: {e}")

                    future.add_done_callback(_on_next_goal_done)
                else:
                    self.get_logger().warn('/next_goal service not ready when goal reached.')
            except Exception as e:
                self.get_logger().error(f'Exception while calling /next_goal: {e}')
           
        else:
            self.get_logger().warn("Failed to compute optimal controls")




        
def main(args=None):
    # rclpy.init(args=args)
    # node = MPPLocalPlannerMPPI()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    rclpy.init(args=args)
    node = MPPLocalPlannerMPPI()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
