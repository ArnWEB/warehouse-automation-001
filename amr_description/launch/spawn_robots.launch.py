from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot.urdf'
    
    return LaunchDescription([
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
