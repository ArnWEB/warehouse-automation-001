#!/usr/bin/env python3
"""
Simple Nav2 Launch for AMR without AMCL (uses static map + odometry)

Usage:
    ros2 launch orchestrator nav2_amr_simple.launch.py namespace:=amr1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = LaunchConfiguration('namespace')
    
    carter_nav_dir = '/home/xelf/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation'
    map_file = os.path.join(carter_nav_dir, 'maps', 'carter_warehouse_navigation.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace (e.g., amr1, amr2)'
        ),
        
        # Static transform from map to odom (robot starts at origin)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            namespace=namespace_arg,
        ),
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace_arg,
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': True
            }]
        ),
        
        # Controller Server (DWB)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace_arg,
            parameters=[{
                'use_sim_time': True,
                'controller_frequency': 20.0,
                'progress_checker_plugin': 'progress_checker',
                'goal_checker_plugin': ['goal_checker'],
                'controller_plugins': ['FollowPath'],
                'progress_checker': {
                    'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': 0.5,
                    'movement_time_allowance': 10.0
                },
                'goal_checker': {
                    'plugin': 'nav2_controller::SimpleGoalChecker',
                    'xy_goal_tolerance': 0.25,
                    'yaw_goal_tolerance': 0.25
                },
                'FollowPath': {
                    'plugin': 'dwb_core::DWBLocalPlanner',
                    'min_vel_x': 0.0,
                    'max_vel_x': 1.0,
                    'max_vel_theta': 1.0,
                    'vx_samples': 10,
                    'vtheta_samples': 20,
                    'sim_time': 1.0,
                    'xy_goal_tolerance': 0.25
                }
            }]
        ),
        
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace_arg,
            parameters=[{
                'use_sim_time': True,
                'expected_planner_frequency': 10.0,
                'planner_plugins': ['GridBased'],
                'GridBased': {
                    'plugin': 'nav2_navfn_planner/NavfnPlanner',
                    'tolerance': 0.5,
                    'use_astar': False,
                    'allow_unknown': True
                }
            }]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace_arg,
            parameters=[{
                'use_sim_time': True,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'bt_loop_duration': 20,
                'default_server_timeout': 40
            }]
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace_arg,
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
