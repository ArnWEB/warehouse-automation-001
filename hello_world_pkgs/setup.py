from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hello_world_pkgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hello_world_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xelf',
    maintainer_email='xelf@todo.todo',
    description='A simple ROS 2 Hello World package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_talker = hello_world_pkgs.hello_talker:main',
            'hello_listener = hello_world_pkgs.hello_listener:main',
        ],
    },
)
