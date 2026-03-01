#!/usr/bin/env python3
"""
Spawn AMR Robots with ros2_control
This launch file spawns robots that respond to cmd_vel
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot_diffdrive.urdf'
    
    return LaunchDescription([
        # Spawn AMR1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'amr1',
                '-file', urdf_file,
                '-x', '-5', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
        
        # Spawn AMR2 (after 2 seconds)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'amr2',
                        '-file', urdf_file,
                        '-x', '-5', '-y', '2', '-z', '0.1'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Spawn AMR3 (after 4 seconds)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'amr3',
                        '-file', urdf_file,
                        '-x', '-5', '-y', '-2', '-z', '0.1'
                    ],
                    output='screen'
                ),
            ]
        ),
    ])
