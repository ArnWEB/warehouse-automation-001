#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Testing Task Executor with Nav2 (Global Namespace) ==="

echo "Starting task executor with Nav2..."
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true &
EXE_PID=$!
sleep 5

echo ""
echo "=== Sending CuOpt plan ==="
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

sleep 3

echo ""
echo "=== Robot State ==="
ros2 topic echo /amr1/robot_state --once

echo ""
echo "=== Check Nav2 action status ==="
ros2 action list

sleep 2
echo ""
echo "Killing process..."
kill $EXE_PID 2>/dev/null
