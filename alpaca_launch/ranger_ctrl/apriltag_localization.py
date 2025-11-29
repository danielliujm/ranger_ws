#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException
from transforms3d import quaternions, affines
import numpy as np


class AprilTagLocalizer(Node):
    def __init__(self):
        super().__init__('apriltag_localizer')
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for camera pose in map frame
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/camera_pose_map', 
            10
        )
        
        # Frame names
        self.map_frame = 'map'
        self.base_frame = 'base'  # AprilTag frame
        self.camera_frame = 'zed_left_camera_optical_frame'
        self.base_link_frame = 'base_link'
        
        # Define the known transform from map to base (AprilTag)
        # Modify these values according to your setup
        self.map_to_base_translation =  [0.018, -0.697, 3.010] # [x, y, z] in meters
        self.map_to_base_rotation = [0.999, 0.002, 0.004, -0.035]  # [x, y, z, w] quaternion
        
        # Timer to periodically compute and publish the camera pose
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('AprilTag Localizer started')
        self.get_logger().info(f'Listening for transform: {self.base_frame} -> {self.camera_frame}')
        self.get_logger().info(f'Publishing camera pose in {self.map_frame} frame')
        
    def timer_callback(self):
        try:    
            # Get transform from base (AprilTag) to camera
            trans = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Extract base to camera transform
            base_to_camera_trans = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            base_to_camera_rot = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]

            base_to_camera_mat = self.transform_to_matrix(
                base_to_camera_trans,
                base_to_camera_rot
            )

            # Convert to transformation matrices
            map_to_base_mat = self.transform_to_matrix(
                self.map_to_base_translation,
                self.map_to_base_rotation
            )
            
            camear_to_baselink_transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_link_frame,
                rclpy.time.Time()
            )

            camera_to_baselink_tras = [
                camear_to_baselink_transform.transform.translation.x,
                camear_to_baselink_transform.transform.translation.y,
                camear_to_baselink_transform.transform.translation.z
            ]

            camera_to_baselink_rot = [
                camear_to_baselink_transform.transform.rotation.x,
                camear_to_baselink_transform.transform.rotation.y,
                camear_to_baselink_transform.transform.rotation.z,
                camear_to_baselink_transform.transform.rotation.w
            ]

            camera_to_baselink_mat = self.transform_to_matrix(
                camera_to_baselink_tras,
                camera_to_baselink_rot
            )

            
            
            # Compute map to camera transform: T_map_camera = T_map_base * T_base_camera
            map_to_baselink_mat = map_to_base_mat@base_to_camera_mat@camera_to_baselink_mat
            
            # Extract translation and rotation from the result
            camera_pos, camera_quat = self.matrix_to_transform(map_to_baselink_mat)
            
            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame
            
            pose_msg.pose.position.x = camera_pos[0]
            pose_msg.pose.position.y = camera_pos[1]
            pose_msg.pose.position.z = camera_pos[2]
            
            pose_msg.pose.orientation.x = camera_quat[0]
            pose_msg.pose.orientation.y = camera_quat[1]
            pose_msg.pose.orientation.z = camera_quat[2]
            pose_msg.pose.orientation.w = camera_quat[3]
            
            self.pose_pub.publish(pose_msg)
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.base_frame} to {self.camera_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            
    def transform_to_matrix(self, translation, quaternion):
        """Convert translation and quaternion to 4x4 transformation matrix"""
        quaternion = quaternions.quat2mat(quaternion)
        mat = affines.compose (translation,quaternion, np.ones(3))
        return mat
    
    def matrix_to_transform(self, matrix):
        """Extract translation and quaternion from 4x4 transformation matrix"""
        translation = matrix[0:3, 3].tolist()
        # quaternion = tf_transformations.quaternion_from_matrix(matrix)
        

        translation, quaternion, _, _ = affines.decompose44(matrix)
        quaternion = quaternions.mat2quat(quaternion)

        return translation, quaternion


def main(args=None):
    rclpy.init(args=args)
    
    node = AprilTagLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()