#!/usr/bin/env python3
"""
Launch file to start Nav2 + Docking Server together
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package paths
    iw_hub_nav_dir = get_package_share_directory("iw_hub_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(
        iw_hub_nav_dir, "params", "iw_hub_navigation_params_amr1.yaml"
    )

    # Start nav2 bringup
    bringup_cmd = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 launch {nav2_bringup_dir}/launch/bringup_launch.py "
            f"map:={iw_hub_nav_dir}/maps/iw_hub_warehouse_navigation.yaml "
            f"use_sim_time:=True "
            f"params_file:={params_file} "
            f"namespace:=amr1 "
            f"use_namespace:=True &",
        ],
        shell=True,
        output="screen",
    )

    # Start docking server
    dock_cmd = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 run opennav_docking opennav_docking "
            f"--ros-args "
            f"--params-file {params_file} "
            f"__ns:=/amr1 &",
        ],
        shell=True,
        output="screen",
    )

    # Configure and activate docking server
    configure_cmd = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "sleep 5 && "
            "source /opt/ros/humble/setup.bash && "
            "ros2 lifecycle set /amr1/docking_server configure && "
            "ros2 lifecycle set /amr1/docking_server activate",
        ],
        shell=True,
        output="screen",
    )

    return LaunchDescription(
        [
            bringup_cmd,
            dock_cmd,
            configure_cmd,
        ]
    )
