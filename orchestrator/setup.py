from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xelf',
    maintainer_email='xelf@todo.todo',
    description='Orchestrator for warehouse automation with Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orchestrator = orchestrator.orchestrator:main',
            'amr_robot = orchestrator.amr_robot:main',
            'fleet_manager = orchestrator.fleet_manager:main',
            'fleet_task_generator = orchestrator.fleet_task_generator:main',
            'fleet_visualization = orchestrator.fleet_visualization:main',
            'fleet_dashboard = orchestrator.fleet_dashboard:main',
            'gazebo_fleet_mover = orchestrator.gazebo_fleet_mover:main',
            'gazebo_velocity_mover = orchestrator.gazebo_velocity_mover:main',
            'simple_mover = orchestrator.simple_mover:main',
            'test_gazebo_mover = orchestrator.test_gazebo_mover:main',
            'robot_state_monitor = orchestrator.robot_state_monitor:main',
            'task_executor = orchestrator.task_executor:main',
            'mock_odom_publisher = orchestrator.mock_odom_publisher:main',
        ],
    },
)
