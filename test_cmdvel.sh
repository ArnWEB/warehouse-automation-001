#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting mock odom..."
ros2 run orchestrator mock_odom_publisher --ros-args -p robot_id:=amr1 &
MOCK_PID=$!
sleep 3

echo "Starting task executor..."
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=false &
EXE_PID=$!
sleep 5

echo "Sending CuOpt plan..."
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

sleep 3
echo "Checking topics..."
ros2 topic list

echo "Killing processes..."
kill $EXE_PID $MOCK_PID 2>/dev/null
