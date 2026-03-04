#!/usr/bin/env python3
# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"),
            "maps",
            "iw_hub_warehouse_navigation.yaml",
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("orchestrator"), "params", "nav2_amr3.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_launch_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": param_dir,
                    "namespace": "amr3",
                    "use_namespace": "True",
                }.items(),
            ),
        ]
    )
