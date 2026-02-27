#!/usr/bin/env python3
"""Launch Gazebo with warehouse world and spawn robots"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    world_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/worlds/warehouse.world'
    urdf_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot.urdf'
    
    return LaunchDescription([
        # Gazebo server (system gzserver with ROS support)
        ExecuteProcess(
            cmd=['gzserver', world_file, 
                 '-slibgazebo_ros_init.so', 
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen'
        ),
        
        # Wait for gzserver to start
        # Then spawn robots
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
    ])
