#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Launching Nav2 for AMR1 ==="
ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr1 namespace:=amr1 &
NAV2_PID=$!
sleep 15

echo "=== Checking Nav2 status ==="
ros2 action list

echo "=== Starting Task Executor ==="
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=false &
EXE_PID=$!
sleep 5

echo "=== Sending CuOpt plan for AMR1 ==="
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

echo "=== Monitoring for 20 seconds ==="
for i in {1..20}; do
    echo "--- Second $i ---"
    ros2 topic echo /amr1/chassis/odom --qos-reliability best_effort --msg nav_msgs/Odometry 2>/dev/null | head -5 || true
    ros2 topic echo /amr1/cmd_vel --qos-reliability best_effort --msg geometry_msgs/Twist 2>/dev/null | head -5 || true
    sleep 1
done

echo "=== Killing processes ==="
kill $EXE_PID $NAV2_PID 2>/dev/null
echo "=== Test Complete ==="
