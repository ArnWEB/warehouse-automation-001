#!/usr/bin/env python3
"""
Launch Gazebo with warehouse world and AMR robots
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gazebo_ros import GazeboRosPackage


def generate_launch_description():
    # Get package paths
    pkg_amr_description = get_package_share_directory('amr_description')
    
    # World file
    world_file = os.path.join(pkg_amr_description, 'worlds', 'warehouse.world')
    
    # URDF file
    urdf_file = os.path.join(pkg_amr_description, 'urdf', 'amr_robot.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    return LaunchDescription([
        # Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['-u', world_file]
        ),
        
        # Spawn AMR1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr1',
            arguments=['-entity', 'amr1', '-x', '-8', '-y', '0', '-z', '0.1', '-Y', '0', '-topic', 'robot_description'],
            output='screen',
        ),
        
        # Spawn AMR2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr2',
            arguments=['-entity', 'amr2', '-x', '-8', '-y', '2', '-z', '0.1', '-Y', '0', '-topic', 'robot_description'],
            output='screen',
        ),
        
        # Spawn AMR3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr3',
            arguments=['-entity', 'amr3', '-x', '-8', '-y', '-2', '-z', '0.1', '-Y', '0', '-topic', 'robot_description'],
            output='screen',
        ),
    ])
