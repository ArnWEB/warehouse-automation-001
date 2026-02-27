from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Order Generator - publishes orders every 5 seconds
        Node(
            package='order_system',
            executable='order_generator',
            name='order_generator',
            output='screen'
        ),
        # Order Listener - logs received orders
        Node(
            package='order_system',
            executable='order_listener',
            name='order_listener',
            output='screen'
        ),
        # CuOpt Bridge - creates plans from orders
        Node(
            package='cuopt_bridge',
            executable='cuopt_bridge',
            name='cuopt_bridge',
            output='screen'
        ),
        # Orchestrator - executes plans
        Node(
            package='orchestrator',
            executable='orchestrator',
            name='orchestrator',
            output='screen'
        ),
        # AMR Robots
        Node(
            package='orchestrator',
            executable='amr_robot',
            name='amr1',
            output='screen',
            arguments=['amr1']
        ),
        Node(
            package='orchestrator',
            executable='amr_robot',
            name='amr2',
            output='screen',
            arguments=['amr2']
        ),
        Node(
            package='orchestrator',
            executable='amr_robot',
            name='amr3',
            output='screen',
            arguments=['amr3']
        )
    ])
