from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    
 
        
    ld = LaunchDescription()
    ld.add_action(ExecuteProcess(
            cmd=[
                '/home/dliujm/venvs/RPEA/bin/python',
                '/home/dliujm/ranger_ws/src/alpaca_detection/alpaca_detection/RPEA_detection_node.py'
            ],
            output='screen'
        ))
    ld.add_action(
       Node (
            package='alpaca_detection',
            executable='SimpleTrack_node',
            name='SimpleTrack_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    )
    return ld