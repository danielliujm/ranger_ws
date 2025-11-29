from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                '/home/dliujm/venvs/RPEA/bin/python',
                '/home/dliujm/ranger_ws/src/alpaca_detection/alpaca_detection/RPEA_detection_node.py'
            ],
            output='screen'
        )
    ])