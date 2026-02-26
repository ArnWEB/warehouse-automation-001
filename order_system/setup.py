from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'order_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/order_system_launch.py', 
            'launch/warehouse_launch.py', 
            'launch/complete_warehouse.launch.py',
            'launch/complete_warehouse_fleet.launch.py',
            'launch/ros_nodes.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xelf',
    maintainer_email='xelf@todo.todo',
    description='Order system for warehouse automation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'order_generator = order_system.order_generator:main',
            'order_listener = order_system.order_listener:main',
        ],
    },
)
