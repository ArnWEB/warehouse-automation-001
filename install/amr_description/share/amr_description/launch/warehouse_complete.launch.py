#!/usr/bin/env python3
"""
Complete warehouse launch - Gazebo + AMR robots
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description'
    world_file = f'{pkg_path}/worlds/warehouse.world'
    urdf_file = f'{pkg_path}/urdf/amr_robot.urdf'
    
    return LaunchDescription([
        # Start Gazebo server with ROS plugins
        ExecuteProcess(
            cmd=['gzserver', world_file, 
                 '-slibgazebo_ros_init.so', 
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen',
            shell=True
        ),
        
        # Start Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            shell=True
        ),
        
        # Wait for Gazebo to start, then spawn robots (after 3 seconds)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'amr1', '-file', urdf_file, '-x', '-5', '-y', '0', '-z', '0.1'],
                    output='screen'
                ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'amr2', '-file', urdf_file, '-x', '-5', '-y', '2', '-z', '0.1'],
                    output='screen'
                ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'amr3', '-file', urdf_file, '-x', '-5', '-y', '-2', '-z', '0.1'],
                    output='screen'
                ),
            ]
        ),
    ])
