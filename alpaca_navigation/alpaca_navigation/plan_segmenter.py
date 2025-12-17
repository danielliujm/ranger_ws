#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor


class PlanSegmenter(Node):
    def __init__(self):
        super().__init__('plan_segmenter')

        # Parameters
        self.declare_parameter('segment_length', 30.0)  # meters

        self.segment_length = (
            self.get_parameter('segment_length').get_parameter_value().double_value
        )

        # Subscriber to the full plan
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        # Publisher for segmented goals
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal',
            10
        )

        # Publisher for visualization markers for each segmented goal
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/goal_markers',
            10
        )

        # Service to publish next goal on demand
        self.next_goal_srv = self.create_service(
            Trigger,
            '/next_goal',
            self.next_goal_callback
        )

        # Internal state
        self.segmented_goals = []  # list[PoseStamped]
        self.current_idx = -1      # index into segmented_goals

        self.get_logger().info(
            f'PlanSegmenter node started. segment_length = {self.segment_length:.3f} m'
        )

    # ===== Helpers =====

    def pose_distance2d(self, p1: PoseStamped, p2: PoseStamped) -> float:
        dx = p1.pose.position.x - p2.pose.position.x
        dy = p1.pose.position.y - p2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def segment_path(self, path: Path):
        """
        Take a Path and return a list of PoseStamped spaced by ~segment_length,
        always including the final pose.
        """
        poses = path.poses
        segmented = []

        if not poses:
            return segmented

        # Always start with the first pose
        segmented.append(poses[0])
        last_kept_pose = poses[0]

        dist_since_last_keep = 0.0

        for i in range(1, len(poses)):
            p = poses[i]
            d = self.pose_distance2d(last_kept_pose, p)
            dist_since_last_keep += d

            if dist_since_last_keep >= self.segment_length:
                segmented.append(p)
                last_kept_pose = p
                dist_since_last_keep = 0.0
                
        # last pose
        if self.pose_distance2d(segmented[-1], poses[-1]) > 1e-6:
            segmented.append(poses[-1])

        return segmented

    def publish_current_goal(self):
        if 0 <= self.current_idx < len(self.segmented_goals):
            goal = self.segmented_goals[self.current_idx]
            self.goal_pub.publish(goal)
            self.get_logger().info(
                f'Published goal {self.current_idx + 1}/{len(self.segmented_goals)}'
            )
            # Update visualization markers so UI shows which goal is current
            self.publish_markers()
        else:
            self.get_logger().warn('No valid current goal to publish.')


    def publish_markers(self):
        """Publish a MarkerArray with a marker at each segmented goal.

        The current goal is highlighted (green), others are blue.
        """
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, ps in enumerate(self.segmented_goals):
            m = Marker()
            m.header.stamp = now
            # use pose header frame if available, otherwise default to map
            m.header.frame_id = ps.header.frame_id if ps.header.frame_id else 'map'
            m.ns = 'segment_goals'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ps.pose.position.x
            m.pose.position.y = ps.pose.position.y
            m.pose.position.z = ps.pose.position.z
            m.pose.orientation = ps.pose.orientation
            # scale
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            # color: current goal green, others blue
            if i == self.current_idx:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                m.color.r = 0.0
                m.color.g = 0.2
                m.color.b = 1.0
                m.color.a = 0.8
            # persistent
            m.lifetime = Duration()
            ma.markers.append(m)

        # publish even if empty (clears previous markers if list empty)
        self.marker_pub.publish(ma)

    # ===== Callbacks =====

    def plan_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty Path on /plan.')
            self.segmented_goals = []
            self.current_idx = -1
            return

        self.get_logger().info(
            f'Received Path with {len(msg.poses)} poses. Segmenting...'
        )

        self.segmented_goals = self.segment_path(msg)
        self.current_idx = 0 if self.segmented_goals else -1

        self.get_logger().info(
            f'Segmented into {len(self.segmented_goals)} goals.'
        )

        # Publish the first goal immediately (if any)
        if self.current_idx >= 0:
            self.publish_current_goal()
        else:
            self.get_logger().warn('No goals after segmentation.')

    def next_goal_callback(self, request, response):
        # Called by another node when current goal is reached.
        if not self.segmented_goals:
            response.success = False
            response.message = 'No active plan/segment goals available.'
            self.get_logger().warn(response.message)
            return response

        if self.current_idx < 0:
            response.success = False
            response.message = 'No current goal index.'
            self.get_logger().warn(response.message)
            return response

        # Move to next index
        if self.current_idx + 1 < len(self.segmented_goals):
            self.current_idx += 1
            self.publish_current_goal()
            response.success = True
            response.message = (
                f'Published next goal {self.current_idx + 1}/{len(self.segmented_goals)}.'
            )
        else:
            # No more goals
            response.success = False
            response.message = 'Last goal already reached. No more goals in this plan.'
            self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PlanSegmenter()
#     executor = MultiThreadedExecutor()
#     try:
#         executor.add_node(node)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         executor.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()


if __name__ == '__main__':
    main()
