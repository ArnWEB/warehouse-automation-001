#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the world file path
    world_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/worlds/warehouse.world'
    
    return LaunchDescription([
        # Launch Gazebo with the world file
        ExecuteProcess(
            cmd=['gzserver', world_file],
            output='screen',
            shell=True
        ),
    ])
