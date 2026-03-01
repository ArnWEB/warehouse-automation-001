#!/usr/bin/env python3
"""
Multi-Robot Nav2 Launch for AMR Fleet

Usage:
    ros2 launch orchestrator nav2_all.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    orchestrator_share = get_package_share_directory('orchestrator')
    amr_description_share = get_package_share_directory('amr_description')
    
    map_file = os.path.join(amr_description_share, 'maps', 'iw_hub_warehouse_navigation.yaml')
    
    amr1_params = os.path.join(orchestrator_share, 'params', 'nav2_amr1.yaml')
    amr2_params = os.path.join(orchestrator_share, 'params', 'nav2_amr2.yaml')
    amr3_params = os.path.join(orchestrator_share, 'params', 'nav2_amr3.yaml')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        ),
        
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Auto-start Nav2 nodes",
        ),
        
        # AMR1 Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "map": map_file,
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": amr1_params,
                "namespace": "amr1",
                "use_namespace": "True",
            }.items(),
        ),
        
        # AMR2 Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "map": map_file,
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": amr2_params,
                "namespace": "amr2",
                "use_namespace": "True",
            }.items(),
        ),
        
        # AMR3 Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "map": map_file,
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": amr3_params,
                "namespace": "amr3",
                "use_namespace": "True",
            }.items(),
        ),
    ])
