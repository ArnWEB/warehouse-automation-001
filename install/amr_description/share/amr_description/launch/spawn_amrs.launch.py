#!/usr/bin/env python3
"""
Launch warehouse world with AMR robots
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package path
    pkg_path = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description'
    
    return LaunchDescription([
        # Spawn AMR1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'amr1',
                '-file', f'{pkg_path}/urdf/amr_robot.urdf',
                '-x', '-5', '-y', '0', '-z', '0.1', '-Y', '0'
            ],
            output='screen'
        ),
        
        # Spawn AMR2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'amr2',
                '-file', f'{pkg_path}/urdf/amr_robot.urdf',
                '-x', '-5', '-y', '2', '-z', '0.1', '-Y', '0'
            ],
            output='screen'
        ),
        
        # Spawn AMR3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'amr3',
                '-file', f'{pkg_path}/urdf/amr_robot.urdf',
                '-x', '-5', '-y', '-2', '-z', '0.1', '-Y', '0'
            ],
            output='screen'
        ),
    ])
