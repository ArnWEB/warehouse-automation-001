#!/bin/bash
cd /home/xelf/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start task executor in background
ros2 run orchestrator task_executor &
EXECUTOR_PID=$!

sleep 2

# Publish mock plan
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [2, 10], \"total_cost\": 15.0}}}'" --once

# Wait for execution
sleep 5

# Kill executor
kill $EXECUTOR_PID 2>/dev/null
