#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString




def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_zed_odom = LaunchConfiguration('use_zed_odom', default='false')

    # use_zed_odom = LaunchConfiguration('use_zed_odom')

    base_bringup_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [os.path.join(get_package_prefix("ranger_bringup"),"share", "ranger_bringup","launch","ranger_mini_v3.launch.xml")],
        )
    )
    
    camera_bringup_launch = IncludeLaunchDescription (
        PythonLaunchDescriptionSource (
            [os.path.join(get_package_prefix ("zed_wrapper"), "share","zed_wrapper","launch", "zed_camera.launch.py")], 

        ),
        launch_arguments  = {'camera_model' : 'zedm', 'publish_tf' : use_zed_odom}.items(),
        condition = IfCondition(use_camera),
        
        
    )
    
    
    lidar_bringup_launch = IncludeLaunchDescription (
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_prefix ("velodyne"), "share", "velodyne_driver", "launch", "velodyne_driver_node-VLP16-launch.py")],
        ),
        # launch_arguments = {'ros-arguments':'--ros-args -r /velodyne_points:=/filtered_cloud'}.items(),
        condition = IfCondition(use_lidar),
    )

    lidar_pointcloud_bringup_launch = IncludeLaunchDescription (
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_prefix ("velodyne"), "share", "velodyne_pointcloud", "launch", "velodyne_transform_node-VLP16-launch.py")],
        ),
        # launch_arguments = {'ros-arguments':'--ros-args -r /velodyne_points:=/filtered_cloud'}.items(),
        condition = IfCondition(use_lidar),
    )

    odom_2_tf_launch = IncludeLaunchDescription (
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_prefix ("alpaca_launch"), "share", "alpaca_launch", "launch", "odom_2_tf_launch.py")],
        )
    )

    lidar_tf_pub = IncludeLaunchDescription (
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_prefix ("alpaca_launch"), "share", "alpaca_launch", "launch", "lidar_tf_pub.launch.py")],
            
        ),
        launch_arguments = {'use_zed_odom' : use_zed_odom}.items(),
    )

    point_2_laserscan_node = Node (
        package = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name = 'pointcloud_to_laserscan_node',
        output = 'screen',
        parameters = [{'target_frame': 'velodyne',
                       'transform_tolerance': 0.01,
                       'min_height': -0.7,
                       'max_height': 0.3,
                       'angle_min': -3.14,
                       'angle_max': 3.14,
                       'angle_increment': 0.0058,
                       'scan_time': 0.1,
                       'range_min': 0.12,
                       'range_max': 20.0,
                       'use_inf': True,
                       'inf_epsilon': 0.1,
                       'output_topic': '/scan',
                       'qos_overrides./scan.publisher.reliability': 'RELIABLE'
                       },
                       ],
        remappings = [('cloud_in', '/velodyne_points')],
        condition = IfCondition(use_lidar),
    )

    # dummy_tf_publisher = ExecuteProcess (
    #     cmd = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'map', 'odom'],
    # )

    pointcloud_filter_node = Node(
        package="alpaca_launch",
        executable="pointcloud_filter",
        name="pointcloud_filter",
        parameters=[{
            "input_topic": "/velodyne_points",
            "output_topic": "/filtered_clouds",
        }],
        )


    cloud_relay_node = Node (
        package = 'topic_tools',
        executable = 'relay',
        name = 'cloud_relay_node',
        output = 'screen',
        parameters = [{'input_topic': '/velodyne_points',
                       'output_topic': '/cloud_in'}],
        
        condition = IfCondition(use_lidar),
    )

    ld = LaunchDescription()

    ld.add_action (DeclareLaunchArgument(
        'use_lidar',
        default_value = 'true',))

    ld.add_action (DeclareLaunchArgument(
        'use_camera',
        default_value = 'true',))

    ld.add_action (DeclareLaunchArgument(
        'use_zed_odom',
        default_value = 'false',))

    ld.add_action (base_bringup_launch)
    ld.add_action (camera_bringup_launch)
    ld.add_action (lidar_bringup_launch)
    ld.add_action (odom_2_tf_launch)
    ld.add_action (lidar_tf_pub)
    ld.add_action (point_2_laserscan_node)
    ld.add_action (cloud_relay_node)
    ld.add_action (pointcloud_filter_node)
    ld.add_action (lidar_pointcloud_bringup_launch)
    # ld.add_action (dummy_tf_publisher)
    # ld.add_action (point_2_laserscan)   
    

    return ld
