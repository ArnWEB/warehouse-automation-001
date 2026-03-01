#!/usr/bin/env python3
"""
Nav2 Launch for AMR with Map-based Navigation (Isaac Sim compatible)

Usage:
    ros2 launch orchestrator nav2_amr.launch.py namespace:=amr1
    ros2 launch orchestrator nav2_amr.launch.py namespace:=amr2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace_arg = LaunchConfiguration('namespace')
    
    carter_nav_dir = '/home/xelf/IsaacSim-ros_workspaces/humble_ws/src/navigation/carter_navigation'
    map_file = os.path.join(carter_nav_dir, 'maps', 'carter_warehouse_navigation.yaml')
    params_file = os.path.join(carter_nav_dir, 'params', 'hospital', 'multi_robot_carter_navigation_params_1.yaml')
    
    carter_launch = os.path.join(carter_nav_dir, 'launch', 'carter_navigation_individual.launch.py')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace (e.g., amr1, amr2)'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(carter_launch),
            launch_arguments={
                'namespace': namespace_arg,
                'use_namespace': 'True',
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': 'True',
            }.items(),
        ),
    ])
