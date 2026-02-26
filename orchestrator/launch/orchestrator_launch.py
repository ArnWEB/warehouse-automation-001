from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Orchestrator - executes CuOpt plans
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
