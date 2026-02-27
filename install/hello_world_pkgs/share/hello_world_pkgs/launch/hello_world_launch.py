from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_world_pkgs',
            executable='hello_talker',
            name='hello_talker',
            output='screen'
        ),
        Node(
            package='hello_world_pkgs',
            executable='hello_listener',
            name='hello_listener',
            output='screen'
        )
    ])
