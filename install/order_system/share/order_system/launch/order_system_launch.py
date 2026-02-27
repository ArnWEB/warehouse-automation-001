from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='order_system',
            executable='order_generator',
            name='order_generator',
            output='screen'
        ),
        Node(
            package='order_system',
            executable='order_listener',
            name='order_listener',
            output='screen'
        )
    ])
