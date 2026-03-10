#!/bin/bash

# End-to-End Test with Real NVIDIA CuOpt Server
# Order → Real CuOpt Optimization → Multi-Robot Execution

set -e

cd /home/ubuntu/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "======================================================================"
echo "STEP 3: Launch Task Executors (Nav2 mode, namespaced)"
echo "======================================================================"

for robot_id in amr1 amr2 amr3; do
    echo "Starting task_executor for $robot_id..."
    ros2 run orchestrator task_executor --ros-args \
      -p robot_id:=$robot_id \
      -p use_nav2:=true \
      -p use_global_namespace:=false \
      > /tmp/${robot_id}_executor.log 2>&1 &
    sleep 2
done

sleep 3
EXECUTOR_COUNT=$(ps aux | grep "task_executor.*robot_id" | grep -v grep | wc -l)
echo "✓ Launched $EXECUTOR_COUNT task executors"

echo ""
# echo "======================================================================"
# echo "STEP 4: Launch CuOpt Client (Real Server Mode)"
# echo "======================================================================"
# echo "This will connect to 43.203.202.86:5000 for real optimization"
# echo ""

# ros2 run cuopt_bridge cuopt_client > /tmp/cuopt_client.log 2>&1 &
# CUOPT_PID=$!
# sleep 3

# if ps -p $CUOPT_PID > /dev/null; then
#     echo "✓ CuOpt client started (PID: $CUOPT_PID)"
#     echo "  Log: /tmp/cuopt_client.log"
# else
#     echo "✗ Failed to start CuOpt client"
#     echo "Check logs: cat /tmp/cuopt_client.log"
#     exit 1
# fi

echo ""
echo "======================================================================"
echo "STEP 5: Wait for System Initialization"
echo "======================================================================"
echo "Waiting 5 seconds for all subscribers to be ready..."
sleep 5

echo ""
echo "======================================================================"
echo "STEP 6: Generate Test Order"
echo "======================================================================"

TEST_ORDER='{
  "order_id": "TEST-E2E-001",
  "priority": 1,
  "items": ["PALLET-A", "PALLET-B", "PALLET-C"],
  "quantities": [5, 3, 2],
  "due_time": '$(date +%s)',
  "status": "pending",
  "timestamp": "'$(date -Iseconds)'"
}'

echo "Publishing test order:"
echo "$TEST_ORDER" | jq '.' 2>/dev/null || echo "$TEST_ORDER"
echo ""

ros2 topic pub /orders std_msgs/String "data: '$TEST_ORDER'" --once
sleep 1

echo "✓ Order published to /orders topic"
echo ""

echo "======================================================================"
echo "STEP 7: Monitor CuOpt Optimization"
echo "======================================================================"
echo "Watching for CuOpt optimization activity (10 seconds)..."
echo ""

# Monitor cuopt_client log for key events
timeout 10 tail -f /tmp/cuopt_client.log 2>/dev/null | grep --line-buffered -E "Connecting|Sending|Waiting|RESPONSE|optimization complete|error" || true

echo ""
echo "======================================================================"
echo "STEP 8: Check Optimized Plan Publication"
echo "======================================================================"
echo "Checking if plan was published to /fleet/cuopt_plan..."
echo ""

timeout 3 ros2 topic echo /fleet/cuopt_plan --once 2>/dev/null | head -20 || echo "⚠️  No plan published yet"

echo ""
echo "======================================================================"
echo "STEP 9: Monitor Robot Execution (30 seconds)"
echo "======================================================================"
echo "Watching task executors respond to plan..."
echo ""

# Check if any robot received the plan
sleep 2
echo "Checking executor logs for plan reception:"
for robot in amr1 amr2 amr3; do
    if grep -q "NEW PLAN" /tmp/${robot}_executor.log 2>/dev/null; then
        echo "  ✓ $robot received plan"
        grep "NEW PLAN\|EXECUTING\|Nav2: Sending goal" /tmp/${robot}_executor.log 2>/dev/null | head -3
    else
        echo "  ✗ $robot did NOT receive plan"
    fi
done

echo ""
echo "Monitoring robot states for 30 seconds..."
for i in {1..30}; do
    printf "\r[%2d sec] " "$i"
    for robot in amr1 amr2 amr3; do
        STATE=$(timeout 0.5 ros2 topic echo /${robot}/robot_state --once 2>/dev/null | grep -o '"busy": [^,]*' | head -1 || echo "busy: ?")
        printf "$robot:$STATE | "
    done
    sleep 1
done
echo ""

echo ""
echo "======================================================================"
echo "STEP 10: Final Status Check"
echo "======================================================================"

echo ""
echo "=== CuOpt Client Log (last 30 lines) ==="
tail -30 /tmp/cuopt_client.log 2>/dev/null || echo "No log available"

echo ""
echo "=== Task Executor Logs Summary ==="
for robot in amr1 amr2 amr3; do
    echo ""
    echo "--- $robot ---"
    grep -E "NEW PLAN|EXECUTING|Nav2.*goal|goal accepted" /tmp/${robot}_executor.log 2>/dev/null | head -5 || echo "No execution logged"
done

echo ""
echo "======================================================================"
echo "STEP 11: Cleanup"
echo "======================================================================"
echo "Killing all test processes..."

kill $CUOPT_PID 2>/dev/null || true
pkill -f task_executor 2>/dev/null || true
sleep 2

echo "✓ All processes stopped"
echo ""

echo "======================================================================"
echo "TEST COMPLETED"
echo "======================================================================"
echo ""
echo "Log files available at:"
echo "  /tmp/cuopt_client.log       - CuOpt client activity"
echo "  /tmp/amr1_executor.log      - AMR1 task execution"
echo "  /tmp/amr2_executor.log      - AMR2 task execution"
echo "  /tmp/amr3_executor.log      - AMR3 task execution"
echo ""
echo "Review logs with:"
echo "  cat /tmp/cuopt_client.log | grep -E 'CUOPT|optimization|error'"
echo "  cat /tmp/amr*_executor.log | grep -E 'NEW PLAN|EXECUTING|Nav2'"
echo ""
