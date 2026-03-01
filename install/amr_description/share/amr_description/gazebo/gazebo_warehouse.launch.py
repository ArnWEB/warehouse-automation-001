#!/usr/bin/env python3
"""
Gazebo Warehouse Launch File (Ignition/Garden)
Launches Gazebo with warehouse world, shelves, and waypoints

Usage:
    ros2 launch amr_description gazebo_warehouse.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_description')
    
    world_file = os.path.join(pkg_amr_description, 'gazebo', 'warehouse.world')
    
    return LaunchDescription([
        # Ignition Gazebo server
        Node(
            package='ros_ign_gazebo',
            executable='ign gazebo',
            name='gazebo',
            output='screen',
            arguments=[world_file, '-s'],
        ),
    ])
