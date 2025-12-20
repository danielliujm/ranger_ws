from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map', default_value='/path/to/map', description='Path to the map yaml/pgm'
    )

    safety_arg = DeclareLaunchArgument(
        'safety', default_value='on', description='Enable safety deadman switch'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/dliujm/ranger_ws/src/nav2_params/navigation_params.yaml',
        description='Full path to the nav2 params file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'
    )

    mppi_pkg_arg = DeclareLaunchArgument(
        'mppi_pkg', default_value='alpaca_navigation', description='Package containing mppi controller node'
    )

    plan_pkg_arg = DeclareLaunchArgument(
        'plan_pkg', default_value='alpaca_navigation', description='Package containing plan segmenter node'
    )

    mppi_node = Node(
        package=LaunchConfiguration('mppi_pkg'),
        executable='mppi_controller_node',
        name='mppi_controller_node',
        output='screen',
        arguments = ['--safety', LaunchConfiguration('safety')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    plan_segmenter_node = Node(
        package=LaunchConfiguration('plan_pkg'),
        executable='plan_segmenter_node',
        name='plan_segmenter_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(params_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(safety_arg)
    ld.add_action(mppi_pkg_arg)
    ld.add_action(plan_pkg_arg)

    ld.add_action(mppi_node)
    ld.add_action(plan_segmenter_node)
    ld.add_action(nav2_bringup_launch)

    return ld
