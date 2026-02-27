from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    world_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/worlds/warehouse.world'
    urdf_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot_diffdrive.urdf'
    
    return LaunchDescription([
        # Gazebo server with ROS plugins
        ExecuteProcess(
            cmd=['gzserver', world_file, 
                 '-slibgazebo_ros_init.so', 
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen'
        ),
        
        # Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        
        # Spawn AMR robots (after Gazebo initializes)
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
        
        # ROS warehouse system nodes
        Node(
            package='order_system',
            executable='order_generator',
            name='order_generator',
            output='screen'
        ),
        Node(
            package='order_system',
            executable='order_listener',
            name='order_listener',
            output='screen'
        ),
        Node(
            package='cuopt_bridge',
            executable='cuopt_bridge',
            name='cuopt_bridge',
            output='screen'
        ),
        Node(
            package='orchestrator',
            executable='orchestrator',
            name='orchestrator',
            output='screen'
        ),
    ])
