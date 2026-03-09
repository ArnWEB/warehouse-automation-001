#!/bin/bash

# Simplified E2E Test: One Order through Real CuOpt

cd /home/ubuntu/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "================================================================"
echo "E2E TEST: Real CuOpt (43.203.202.86:5000)"
echo "================================================================"
echo ""

# Clean up
echo "1. Cleanup..."
pkill -f task_executor 2>/dev/null || true
pkill -f cuopt_client.py 2>/dev/null || true
sleep 2

# Launch task executors
echo "2. Launching task executors (Nav2 mode)..."
for id in amr1 amr2 amr3; do
    ros2 run orchestrator task_executor --ros-args \
      -p robot_id:=$id \
      -p use_nav2:=true \
      -p use_global_namespace:=false \
      > /tmp/${id}_exec.log 2>&1 &
    sleep 2
done
echo "   ✓ Task executors launched"

# Launch CuOpt client
echo "3. Launching CuOpt client (real server)..."
python3 cuopt_bridge/cuopt_bridge/cuopt_client.py > /tmp/cuopt.log 2>&1 &
CUOPT_PID=$!
sleep 4

if ps -p $CUOPT_PID > /dev/null; then
    echo "   ✓ CuOpt client running (PID: $CUOPT_PID)"
    tail -5 /tmp/cuopt.log
else
    echo "   ✗ CuOpt client failed"
    exit 1
fi

# Wait for subscribers
echo ""
echo "4. Waiting for system ready (5 sec)..."
sleep 5

# Send test order
echo ""
echo "5. Publishing test order to /cuopt/trigger topic..."
echo "   Tasks: [10, 20, 30] (waypoints)"

ros2 topic pub /cuopt/trigger std_msgs/Int32MultiArray "data: [10, 20, 30]" --once
sleep 2

echo ""
echo "6. Monitoring CuOpt activity (15 seconds)..."
timeout 15 tail -f /tmp/cuopt.log 2>/dev/null | grep --line-buffered -E "Connecting|optimization|RESPONSE|error|vehicle_data" || true

echo ""
echo "7. Checking if plan was published..."
timeout 2 ros2 topic echo /fleet/cuopt_plan --once 2>/dev/null | jq '.' 2>/dev/null || echo "No plan yet"

echo ""
echo "8. Checking robot execution..."
for id in amr1 amr2 amr3; do
    if grep -q "NEW PLAN" /tmp/${id}_exec.log 2>/dev/null; then
        echo "   ✓ $id: Plan received and executing"
        grep "NEW PLAN\|EXECUTING\|Nav2.*goal" /tmp/${id}_exec.log | head -2
    else
        echo "   - $id: No plan received"
    fi  
done

echo ""
echo "9. Monitoring robots for 20 seconds..."
for i in {1..20}; do
    printf "\r  [%2d sec] " "$i"
    for id in amr1 amr2 amr3; do
        busy=$(timeout 0.5 ros2 topic echo /${id}/robot_state --once 2>/dev/null | grep -o '"busy":[^,}]*' || echo "busy:?")
        printf "$id:$busy | "
    done
    sleep 1
done
echo ""

echo ""
echo "================================================================"
echo "TEST COMPLETE"
echo "================================================================"
echo "Logs:"
echo "  CuOpt: /tmp/cuopt.log"
echo "  Robots: /tmp/amr{1,2,3}_exec.log"
echo ""
echo "View CuOpt activity:"
echo "  cat /tmp/cuopt.log | grep -A10 'CUOPT RESPONSE'"
echo ""
echo "View robot execution:"
echo "  grep -h 'NEW PLAN\|EXECUTING' /tmp/amr*_exec.log"
echo ""

# Cleanup
pkill -f task_executor 2>/dev/null || true
pkill -f cuopt_client.py 2>/dev/null || true
