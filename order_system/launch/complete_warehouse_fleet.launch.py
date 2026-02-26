#!/usr/bin/env python3
"""
Complete Warehouse Launch - Gazebo + Fleet System
Runs everything in correct order:
1. Start Gazebo with warehouse
2. Spawn AMR robots
3. Start fleet management system
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    world_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/worlds/warehouse.world'
    urdf_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot.urdf'
    
    return LaunchDescription([
        # ===== SECTION 1: GAZEBO =====
        ExecuteProcess(
            cmd=['gzserver', world_file, 
                 '-slibgazebo_ros_init.so', 
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen',
            name='gazebo_server'
        ),
        
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            name='gazebo_client'
        ),
        
        # ===== SECTION 2: SPAWN ROBOTS (after 3 seconds) =====
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
        
        # ===== SECTION 3: FLEET SYSTEM (after 8 seconds) =====
        TimerAction(
            period=8.0,
            actions=[
                # Fleet Task Generator
                Node(
                    package='orchestrator',
                    executable='fleet_task_generator',
                    name='fleet_task_generator',
                    output='screen'
                ),
                
                # Fleet Manager
                Node(
                    package='orchestrator',
                    executable='fleet_manager',
                    name='fleet_manager',
                    output='screen'
                ),
                
                # CuOpt Client
                Node(
                    package='cuopt_bridge',
                    executable='cuopt_client',
                    name='cuopt_client',
                    output='screen'
                ),
                
                # Task Executors (one per robot)
                Node(
                    package='orchestrator',
                    executable='task_executor',
                    name='task_executor_amr1',
                    output='screen'
                ),
                Node(
                    package='orchestrator',
                    executable='task_executor',
                    name='task_executor_amr2',
                    output='screen'
                ),
                Node(
                    package='orchestrator',
                    executable='task_executor',
                    name='task_executor_amr3',
                    output='screen'
                ),
                
                # Dashboard
                Node(
                    package='orchestrator',
                    executable='fleet_dashboard',
                    name='fleet_dashboard',
                    output='screen'
                ),
            ]
        ),
    ])
