from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cuopt_bridge',
            executable='cuopt_bridge',
            name='cuopt_bridge',
            output='screen'
        )
    ])
