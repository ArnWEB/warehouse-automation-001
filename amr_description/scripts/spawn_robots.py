#!/usr/bin/env python3
"""Spawn robots in Gazebo"""
import subprocess
import time
import os

pkg_path = '/home/xelf/warehouse-automation/install/amr_description/share/amr_description'
urdf_file = f'{pkg_path}/urdf/amr_robot.urdf'

# Positions for 3 robots
positions = [
    '-x -5 -y 0 -z 0.1',
    '-x -5 -y 2 -z 0.1', 
    '-x -5 -y -2 -z 0.1'
]

for i, pos in enumerate(positions, 1):
    cmd = f'ros2 run gazebo_ros spawn_entity.py -entity amr{i} -file {urdf_file} {pos}'
    print(f'Spawning amr{i}...')
    subprocess.run(cmd, shell=True)
    time.sleep(1)

print('Done!')
