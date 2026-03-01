#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Launching Nav2 for AMR1 (will run for 30 seconds) ==="
timeout 30 ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr1 namespace:=amr1 2>&1
