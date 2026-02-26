#!/usr/bin/env python3
"""
Simple Gazebo Launch - Warehouse World
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the workspace path
    workspace = '/home/xelf/warehouse-automation'
    
    world_file = os.path.join(workspace, 'install', 'amr_description', 'share', 'amr_description', 'worlds', 'warehouse.world')
    
    return LaunchDescription([
        # Start Gazebo server
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gzserver',
            output='screen',
            arguments=[world_file, '--verbose']
        ),
        
        # Launch Gazebo client (GUI)
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gzclient',
            output='screen',
        ),
    ])
