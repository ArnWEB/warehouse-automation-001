#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Test: Global Namespace Nav2 ==="
echo "Starting task executor..."
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=true &
EXE_PID=$!
sleep 5

echo "Sending CuOpt plan..."
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

sleep 5

echo ""
echo "=== Test: Namespaced Nav2 ==="
echo "Would use: -p use_nav2:=true -p use_global_namespace:=false"
echo "Example: ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=false"

kill $EXE_PID 2>/dev/null
