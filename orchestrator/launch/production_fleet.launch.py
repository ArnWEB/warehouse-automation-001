#!/usr/bin/env python3
"""
Production Warehouse Launch - Fleet Manager + CuOpt + Executors
Complete orchestration system with Nav2 integration ready

Usage:
    ros2 launch orchestrator production_fleet.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    num_robots = 3
    use_nav2 = False
    
    nodes = [
        # Fleet Task Generator - Creates tasks
        Node(
            package='orchestrator',
            executable='fleet_task_generator',
            name='fleet_task_generator',
            output='screen',
        ),
        
        # Fleet Manager - Central brain
        Node(
            package='orchestrator',
            executable='fleet_manager',
            name='fleet_manager',
            output='screen'
        ),
        
        # Robot State Monitor
        Node(
            package='orchestrator',
            executable='robot_state_monitor',
            name='robot_state_monitor',
            output='screen',
        ),
        
        # CuOpt Client
        Node(
            package='cuopt_bridge',
            executable='cuopt_client',
            name='cuopt_client',
            output='screen',
        ),
        
        # Fleet Visualization
        Node(
            package='orchestrator',
            executable='fleet_visualization',
            name='fleet_visualization',
            output='screen',
        ),
        
        # Fleet Dashboard - Shows execution flow
        Node(
            package='orchestrator',
            executable='fleet_dashboard',
            name='fleet_dashboard',
            output='screen',
        ),
        
        # Gazebo Fleet Mover - Try to move robots
        Node(
            package='orchestrator',
            executable='gazebo_fleet_mover',
            name='gazebo_fleet_mover',
            output='screen',
        ),
    ]
    
    for i in range(1, num_robots + 1):
        nodes.append(
            Node(
                package='orchestrator',
                executable='task_executor',
                name=f'task_executor_amr{i}',
                output='screen',
            )
        )
    
    return LaunchDescription(nodes)
