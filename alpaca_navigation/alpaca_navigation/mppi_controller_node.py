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
import sys
import argparse
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, Point
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32
from vision_msgs.msg import Detection3D, Detection3DArray
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose, do_transform_point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from alpaca_navigation.viz_utils import *

import os
import termios
import tty
import select 



EPSILON = 1e-12


class NonBlockingStdin:
    """Lightweight non-blocking stdin reader for the deadman switch."""

    def __init__(self):
        self.fd = None
        self.old_termios = None
        self.enabled = sys.stdin.isatty()
        if self.enabled:
            self.fd = sys.stdin.fileno()
            self.old_termios = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def restore(self):
        if self.enabled and self.old_termios is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_termios)

    def get_key(self):
        if not self.enabled:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if ready:
            return sys.stdin.read(1)
        return None

class MPPLocalPlannerMPPI(Node):
    def __init__(self, pose_source_override=None, safety = 'off'):
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
        self.get_logger().info(f'Horizion lenght :{self.controller.horizon}')
        self.counter = 0
        self.filtered_action = torch.tensor([0.0, 0.0,0.0], dtype=torch.float32).to(self.device)
        self.alpha = 0.7
    
        

        self.current_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)  # [x, y, yaw]
        self.robot_velocity = torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device)
        self.previous_robot_state = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device)

        # self.agent_states = {i: torch.tensor([10.0, 10.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        # self.agent_velocities = {i: torch.tensor([0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}
        # self.previous_agent_states = {i: torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32).to(self.device) for i in range(ACTIVE_AGENTS)}

        self.agents = {}
        self.agents_last_seen = {} 
        self.agent_velocities = {}
        self.agent_time = 0.0
        self.prev_agent_time = 0.0

        self.prev_control = 0.0
        self.control_variation = 0.0
        self.speak = False
        self.last_speak_time = 0.0
    
        self.current_robot_pose = None

        # pose source parameter: 'amcl' (default) or 'odom'
        if pose_source_override is not None:
            self.pose_source = str(pose_source_override).lower()
            self.get_logger().info(f"Pose source override provided: '{self.pose_source}'")
        else:
            self.declare_parameter('pose_source', 'amcl')
            try:
                self.pose_source = str(self.get_parameter('pose_source').value).lower()
            except Exception:
                self.pose_source = 'amcl'

        # subscribe to current position based on pose_source
        if self.pose_source == 'odom':
            self.get_logger().info("Pose source set to 'odom' - subscribing to /odom")
            self.robot_pose_subscriber = self.create_subscription(Odometry, '/odom', self.robot_pose_cb, 10)
        else:
            self.get_logger().info("Pose source set to 'amcl' - subscribing to /amcl_pose")
            self.robot_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.robot_pose_cb, 10)

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

        # tf buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # publisher for visualizing rollouts (MarkerArray)
        self.rollouts_pub = self.create_publisher(MarkerArray, '/mppi_rollouts', 10)

        # visualization utils
        self.viz_tool = VisualizationUtils(self)
        
        # safety no movement if safety is on 
        self.safety = (safety == 'on')

        # Start a separate thread to monitor keyboard input deadman switch
        self.space_timeout = 0.15
        self.last_space_time = 0.0
        self.running = False
        self.keyboard = None
        self.key_thread = None
        if self.safety:
            self.running = True
            self.keyboard = NonBlockingStdin()
            self.key_thread = Thread(target=self.key_loop, daemon=True)
            self.key_thread.start()
    
    def key_loop(self):
        while self.running and rclpy.ok():
            if self.keyboard is None:
                time.sleep(0.05)
                continue
            key = self.keyboard.get_key()
            if key == " ":
                self.last_space_time = time.time()
            time.sleep(0.01)

    def deadman_active(self):
        """Return True when motion is allowed under the safety deadman switch."""
        if not self.safety:
            return True
        return (time.time() - self.last_space_time) <= self.space_timeout

    def publish_twist_command(self, twist_msg):
        """Publish `twist_msg` or zeros depending on the deadman state."""
        if not self.safety:
            self.cmd_vel_pub.publish(twist_msg)
            return

        if self.deadman_active():
            self.cmd_vel_pub.publish(twist_msg)
        else:
            self.cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        self.running = False
        try:
            thread = getattr(self, "key_thread", None)
            if thread is not None and thread.is_alive():
                thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            keyboard = getattr(self, "keyboard", None)
            if keyboard is not None:
                keyboard.restore()
        except Exception:
            pass
        super().destroy_node()


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
        self.prev_agent_time = self.agent_time
        self.agent_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.agents_last_seen = self.agents.copy()
        
        for det in msg.detections:
            id = int(det.results[0].hypothesis.class_id)
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y

            x_map, y_map = self.convert_to_map_frame (x, y, msg.header.frame_id)

            # self.get_logger().info (f'detected human in map frame {x_map}, {y_map} with id {id} ')

            if x_map is None or y_map is None:
                self.get_logger().warn (f' No detections or failed to convert to map frame')
                continue

            self.agents[id] = torch.tensor ([x_map, y_map, 0.0], dtype=torch.float32).to(self.device)
            if id in self.agents_last_seen:
                dt = self.agent_time - self.prev_agent_time
                if dt > 0:
                    prev_pos = self.agents_last_seen[id]
                    curr_pos = self.agents[id]
                    velocity = (curr_pos[:2] - prev_pos[:2])/dt

                    if velocity.norm() < 0.1:
                        velocity = torch.tensor ([0.0, 0.0], dtype=torch.float32).to(self.device)

                    # self.get_logger().info (f'human {id} position: {curr_pos[:2]}, velocity: {velocity}')
                    self.agent_velocities[id] = velocity
            else:
                self.agent_velocities[id] = torch.tensor ([0.0, 0.0], dtype=torch.float32).to(self.device)
        
        
    def convert_to_map_frame (self, x, y, frame_id):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0
        try: 
            transform = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
            transformed_point = do_transform_point(point, transform)
            return transformed_point.point.x, transformed_point.point.y
        
        except Exception as e:
            self.get_logger().warn (f'Failed to transform point from {frame_id} to map frame: {e}')
            return None, None



    def robot_pose_cb (self, msg):
        self.get_logger().debug('found robot pose')
        self.current_robot_pose = msg

        if self.current_robot_pose is None:
            self.get_logger().warn("Can't find robot pose")
            return

        # If pose_source is odom, msg will be an Odometry message. Use it to
        # update current_state immediately and use odom twist for velocity.
        if getattr(self, 'pose_source', 'amcl') == 'odom':
            try:
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                yaw = 2.0 * math.atan2(qz, qw)

                self.current_state = torch.tensor([
                    px,
                    py,
                    yaw,
                ], dtype=torch.float32).to(self.device)

                # If odometry contains twist information, use it to set robot_velocity
                try:
                    linx = msg.twist.twist.linear.x
                    liny = msg.twist.twist.linear.y
                except Exception:
                    linx = 0.0
                    liny = 0.0

                self.robot_velocity = torch.tensor([linx, liny], dtype=torch.float32).to(self.device)
                return
            except Exception as e:
                self.get_logger().warn(f'Failed to parse Odometry message: {e}')

        # Fallback / AMCL (PoseWithCovarianceStamped) handling
        try:
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
        except Exception as e:
            self.get_logger().warn(f'Failed to parse pose message: {e}')

    def local_costmap_cb (self, msg):
        # self.get_logger().info ('received local costmap')
        self.controller.set_local_costmap (msg)
        
    def publish_rollouts(self):
        """Publish current rollouts as a MarkerArray for real-time visualization.

        Markers are published in the `map` frame as LINE_STRIP markers where each
        marker corresponds to a single sampled rollout trajectory.
        """
        if getattr(self, 'rollouts', None) is None:
            return

        try:
            rollouts = self.rollouts

            # convert torch -> numpy if needed
            if isinstance(rollouts, torch.Tensor):
                rollouts = rollouts.detach().cpu().numpy()

            # handle possible batch dimension added elsewhere
            if rollouts.ndim == 4 and rollouts.shape[0] == 1:
                # shape like (1, nsamples, horizon, dim) -> squeeze batch
                rollouts = rollouts[0]

            # At this point common shapes include:
            #  - (nsamples, horizon, dim)
            #  - (horizon, nsamples, dim)
            # Normalize to (horizon, nsamples, dim)
            if rollouts.ndim == 3:
                a, b, c = rollouts.shape
                # If first axis is likely nsamples (large), and second is likely horizon (smaller), transpose
                if a > b:
                    # assume (nsamples, horizon, dim) -> transpose
                    rollouts = rollouts.transpose(1, 0, 2)
            else:
                # unsupported shape
                return

            horizon, nsamples, dim = rollouts.shape

            rollout_1 = rollouts[:,100,:]
            diff = rollouts - np.expand_dims (rollout_1, axis = 1)
            dist = np.linalg.norm (diff, axis=2)
            mean_dist = np.mean (dist)
            self.get_logger().info (f'rollout mean dist from 100th rollout: {mean_dist}')

            if dim < 2:
                return

            marker_array = MarkerArray()
            for n in range(nsamples):
                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'mppi_rollouts'
                m.id = n
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.03
                m.color.r = 0.0
                m.color.g = 0.6
                m.color.b = 1.0
                m.color.a = 0.8
                m.pose.orientation.w = 1.0

                pts = []
                for h in range(horizon):
                    p = Point()
                    # Use first two dims as X,Y. If a third exists, use as Z.
                    p.x = float(rollouts[h, n, 0])
                    p.y = float(rollouts[h, n, 1])
                    p.z = float(rollouts[h, n, 2]) if dim > 2 else 0.0
                    pts.append(p)

                m.points = pts
                marker_array.markers.append(m)

            self.rollouts_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().warn(f'publish_rollouts exception: {e}')


            

    def plan_and_publish(self):
        
        if self.curr_goal is None:
            # self.get_logger().info('waiting for goal ...')
            return
        # self.get_logger().info(f'the goal is {self.curr_goal}, the controller running at HZ {HZ}')

        self.counter += 1


        
        
        #############################################################################
        # self.get_logger().info (f'current robot heading: {self.current_state[2].item()} rad, current desired heading: {math.atan2(self.curr_goal[1]-self.current_state[1], self.curr_goal[0]-self.current_state[0])} rad')
        ############################################################################
        
        if self.counter == 3:
            self.previous_robot_state = self.current_state
            
        if torch.any(self.previous_robot_state):
            self.robot_velocity = (self.current_state[:2] - self.previous_robot_state[:2])/HZ
            self.previous_robot_state = self.current_state
        





        ############################## debug: add fake human ###############################################
        # self.agents[1] = torch.tensor ([4.0, 0.0, 0.0], dtype=torch.float32).to(self.device)
        # self.agent_velocities[1] = torch.tensor ([-0.1, 0.0], dtype=torch.float32).to(self.device)
        ###########################################################################################################


        action, self.rollouts, self.costs, termination = self.controller.compute_control(
            self.current_state, self.previous_robot_state, self.robot_velocity, self.agents, self.agents_last_seen, self.agent_velocities
        )


        self.cost_pub.publish (Float32(data = torch.min (self.costs).item()))
        
        # self.rollouts = self.rollouts.unsqueeze(0)

        # publish rollouts for visualization

        self.viz_tool.visualize_rollouts (self.rollouts, self.costs)

        # try:
        #     self.publish_rollouts()
        # except Exception as e:
        #     self.get_logger().warn(f'publish_rollouts failed: {e}')

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


            # self.get_logger().info(f'x y z {x_effort} {y_effort} {action[2].item()}')

            
            twist_stamped.linear.x = x_effort 
            twist_stamped.linear.y = y_effort
            twist_stamped.angular.z = action[2].item()

            self.control_variation += np.sqrt((self.prev_control - y_effort)**2)
            self.prev_control = y_effort
           
            # twist_stamped.linear.x = 0.0
            # twist_stamped.linear.y = 0.0
            # twist_stamped.angular.z = 0.0

            # twist_stamped.angular.z = action[1].item() #min(action[1].item(), VMAX)

            self.publish_twist_command(twist_stamped)
            
                      
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
            self.publish_twist_command(Twist())
           
        else:
            self.get_logger().warn("Failed to compute optimal controls")
            self.publish_twist_command(Twist())




# helpers 
def create_rollout_video(self, output_file="rollout_animation.mp4"):
    if self.min_rollouts is None:
        print("Min rollout data not loaded. Please call `read_data()` first.")
        return
    min_rollouts_xy = self.min_rollouts[:, :5, :2]
    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Min Rollout Trajectories (5 Time Horizons)")
    ax.grid(True)
    min_lines = [ax.plot([], [], color="red", linestyle=":", linewidth=1)[0]
                for _ in range(min_rollouts_xy.shape[1])]  # 5 rollouts in red
    # Set axis limits based on the data
    x_min = np.min(min_rollouts_xy[:, :, 0])
    x_max = np.max(min_rollouts_xy[:, :, 0])
    y_min = np.min(min_rollouts_xy[:, :, 1])
    y_max = np.max(min_rollouts_xy[:, :, 1])
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    # Add a legend
    ax.legend(handles=[
        plt.Line2D([], [], color="red", linestyle=":", label="Min Rollouts")
    ])
    # Function to update the plot for each frame
    def update(frame):
        for i in range(len(min_lines)):
            # Update min rollout lines up to the current frame
            min_lines[i].set_data(min_rollouts_xy[:frame, i, 0], min_rollouts_xy[:frame, i, 1])
        return min_lines
    # Create the animation
    ani = animation.FuncAnimation(
        fig, update, frames=min_rollouts_xy.shape[0], interval=50, blit=True
    )


        
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument ('--safety', choices = ['on', 'off'], default = 'off')
    parser.add_argument('--pose-source', choices=['amcl', 'odom'], default='amcl',
                        help="Pose source for the controller (amcl or odom). If provided, overrides parameter.)")
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = MPPLocalPlannerMPPI(pose_source_override=parsed.pose_source, safety = parsed.safety)
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
