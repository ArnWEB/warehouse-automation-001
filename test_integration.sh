#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/xelf/warehouse-automation-001/install/setup.bash

echo "Sending CuOpt plan for AMR1..."
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

sleep 2

echo ""
echo "=== Checking Robot State ==="
ros2 topic echo /amr1/robot_state --once

echo ""
echo "=== Checking cmd_vel ==="
ros2 topic echo /amr1/cmd_vel --once
