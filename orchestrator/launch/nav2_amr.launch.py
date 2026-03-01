#!/usr/bin/env python3
"""
Nav2 Launch for AMR with Map-based Navigation (Isaac Sim compatible)

Usage:
    ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr1
    ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr2
    ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    
    orchestrator_share = get_package_share_directory('orchestrator')
    amr_description_share = get_package_share_directory('amr_description')
    
    nav2_params = os.path.join(orchestrator_share, 'params', 'nav2_isaacsim.yaml')
    map_file = os.path.join(amr_description_share, 'maps', 'iw_hub_warehouse_navigation.yaml')
    
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='amr1',
            description='Robot ID (amr1, amr2, amr3)'
        ),
        
        # Lifecycle Manager - auto-activate Nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }]
        ),
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': True
            }]
        ),
        
        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=robot_id,
            output='screen',
            parameters=[nav2_params]
        ),
        
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_id,
            output='screen',
            parameters=[nav2_params]
        ),
        
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_id,
            output='screen',
            parameters=[nav2_params]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_id,
            output='screen',
            parameters=[nav2_params]
        ),
        
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_id,
            output='screen',
            parameters=[nav2_params]
        ),
    ])
    
    return ld
