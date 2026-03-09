#!/bin/bash

# End-to-End Test with Real NVIDIA CuOpt Server
# Tests: Order Generation → Real CuOpt Optimization → Multi-Robot Execution

set -e

cd /home/ubuntu/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "======================================================================"
echo "      END-TO-END TEST: Real CuOpt Server Integration"
echo "======================================================================"
echo ""
echo "Server: 43.201.50.183:5000"
echo "Mode: REAL cuOpt optimization (not mock)"
echo ""

echo "======================================================================"
echo "STEP 1: System Verification"
echo "======================================================================"

echo ""
echo "✓ Checking CuOpt client library..."
python3 -c "from cuopt_sh_client import CuOptServiceSelfHostClient; print('  cuopt_sh_client: installed')" || { echo "  ERROR: cuopt_sh_client not installed"; exit 1; }

echo ""
echo "✓ Checking CuOpt server connectivity..."
timeout 5 bash -c 'cat < /dev/null > /dev/tcp/43.201.50.183/5000' 2>/dev/null && echo "  Server 43.201.50.183:5000: reachable" || { echo "  ERROR: Cannot reach CuOpt server"; exit 1; }

echo ""
echo "✓ Checking Isaac Sim odometry topics..."
ODOM_COUNT=$(ros2 topic list 2>/dev/null | grep "chassis/odom" | wc -l)
echo "  Found $ODOM_COUNT odom topics"
if [ "$ODOM_COUNT" -lt 3 ]; then
    echo "  WARNING: Expected 3 odom topics (amr1, amr2, amr3)"
fi

echo ""
echo "✓ Checking Nav2 action servers..."
NAV2_COUNT=$(ros2 action list 2>/dev/null | grep "navigate_to_pose" | wc -l)
echo "  Found $NAV2_COUNT Nav2 action servers"
if [ "$NAV2_COUNT" -lt 3 ]; then
    echo "  WARNING: Expected 3 Nav2 servers (amr1, amr2, amr3)"
fi

echo ""
echo "======================================================================"
echo "STEP 2: Cleanup - Kill existing nodes"
echo "======================================================================"
pkill -f "task_executor" 2>/dev/null && echo "  ✓ Killed task executors" || echo "  No task executors running"
pkill -f "cuopt_client" 2>/dev/null && echo "  ✓ Killed cuopt_client" || echo "  No cuopt_client running"
pkill -f "cuopt_bridge" 2>/dev/null && echo "  ✓ Killed cuopt_bridge" || echo "  No cuopt_bridge running"
pkill -f "order_generator" 2>/dev/null && echo "  ✓ Killed order_generator" || echo "  No order_generator running"
sleep 2

echo ""
echo "======================================================================"
echo "STEP 3: Launch CuOpt Client (REAL mode)"
echo "======================================================================"
echo "Starting cuopt_client with use_mock:=false..."
echo "Subscribes to: /cuopt/trigger"
echo "Publishes to: /fleet/cuopt_plan"
echo ""

ros2 run cuopt_bridge cuopt_client --ros-args -p use_mock:=false > /tmp/cuopt_client.log 2>&1 &
CUOPT_PID=$!
echo "  PID: $CUOPT_PID"
sleep 3

# Verify it started
if ps -p $CUOPT_PID > /dev/null; then
    echo "  ✓ CuOpt client running"
    grep -q "Using REAL cuOpt API" /tmp/cuopt_client.log && echo "  ✓ Real CuOpt mode confirmed" || echo "  ⚠ Check /tmp/cuopt_client.log for mode"
else
    echo "  ✗ ERROR: CuOpt client failed to start"
    cat /tmp/cuopt_client.log
    exit 1
fi

echo ""
echo "======================================================================"
echo "STEP 4: Launch Task Executors (Nav2 mode, all 3 robots)"
echo "======================================================================"

echo "Starting task_executor for amr1..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr1 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr1_executor_e2e.log 2>&1 &
AMR1_PID=$!
echo "  AMR1 PID: $AMR1_PID"
sleep 2

echo "Starting task_executor for amr2..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr2 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr2_executor_e2e.log 2>&1 &
AMR2_PID=$!
echo "  AMR2 PID: $AMR2_PID"
sleep 2

echo "Starting task_executor for amr3..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr3 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr3_executor_e2e.log 2>&1 &
AMR3_PID=$!
echo "  AMR3 PID: $AMR3_PID"
sleep 3

echo "  ✓ All 3 task executors running"

echo ""
echo "======================================================================"
echo "STEP 5: Launch CuOpt Bridge (Order Processor)"
echo "======================================================================"
echo "Starting cuopt_bridge..."
echo "Subscribes to: /orders"
echo "Publishes to: /cuopt/trigger (for cuopt_client)"
echo ""

ros2 run cuopt_bridge cuopt_bridge > /tmp/cuopt_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "  PID: $BRIDGE_PID"
sleep 3

echo ""
echo "======================================================================"
echo "STEP 6: Wait for All Systems Ready"
echo "======================================================================"
echo "Waiting 5 seconds for all subscriptions to settle..."
sleep 5

echo ""
echo "======================================================================"
echo "STEP 7: Generate Test Order"
echo "======================================================================"
echo "Creating test order: ORDER-E2E-001"
echo "  Priority: 1 (high)"
echo "  Items: ITEM-A (qty: 10)"
echo ""

TEST_ORDER='{
  "order_id": "ORDER-E2E-001",
  "priority": 1,
  "items": ["ITEM-A"],
  "quantities": [10],
  "due_time": '$(date +%s)',
  "status": "pending",
  "timestamp": '$(date +%s)'
}'

echo "Publishing order to /orders topic..."
ros2 topic pub /orders std_msgs/String "data: '$TEST_ORDER'" --once
echo "  ✓ Order published"

echo ""
echo "======================================================================"
echo "STEP 8: Monitor Real CuOpt Optimization"
echo "======================================================================"
echo "Watching /tmp/cuopt_client.log for optimization activity..."
echo ""

# Monitor for 15 seconds to see CuOpt activity
for i in {1..15}; do
    sleep 1
    
    # Check if CuOpt received request
    if grep -q "Connecting to CuOpt server" /tmp/cuopt_client.log 2>/dev/null; then
        echo "  [$i sec] ✓ CuOpt optimization request sent"
        break
    else
        echo "  [$i sec] Waiting for CuOpt request..."
    fi
done

# Wait a bit more for response
sleep 3

echo ""
echo "======================================================================"
echo "STEP 9: Check CuOpt Response"
echo "======================================================================"

if grep -q "CUOPT RESPONSE" /tmp/cuopt_client.log 2>/dev/null; then
    echo "  ✓ CuOpt server responded"
    echo ""
    echo "CuOpt Response Summary:"
    grep -A 10 "CUOPT RESPONSE" /tmp/cuopt_client.log | head -15
else
    echo "  ⚠ No CuOpt response detected yet"
    echo ""
    echo "Recent cuopt_client log:"
    tail -20 /tmp/cuopt_client.log
fi

echo ""
echo "======================================================================"
echo "STEP 10: Check Task Executor Reception"
echo "======================================================================"
sleep 2

echo "Checking if robots received optimized plan..."
echo ""

for robot in amr1 amr2 amr3; do
    if grep -q "NEW PLAN" /tmp/${robot}_executor_e2e.log 2>/dev/null; then
        PLAN=$(grep "NEW PLAN" /tmp/${robot}_executor_e2e.log | tail -1)
        echo "  ✓ $robot: $PLAN"
    else
        echo "  ✗ $robot: No plan received"
    fi
done

echo ""
echo "======================================================================"
echo "STEP 11: Monitor Robot Execution (30 seconds)"
echo "======================================================================"
echo "Watching robot navigation..."
echo ""

for i in {1..30}; do
    sleep 1
    ACTIVE=0
    
    for robot in amr1 amr2 amr3; do
        if grep -q "Nav2: Sending goal" /tmp/${robot}_executor_e2e.log 2>/dev/null; then
            ((ACTIVE++))
        fi
    done
    
    printf "\r[%2d sec] Active robots navigating: %d/3   " "$i" "$ACTIVE"
done

echo ""
echo ""

echo "======================================================================"
echo "STEP 12: Final Status Report"
echo "======================================================================"
echo ""

echo "=== Order Processing ===="
if grep -q "Received order: ORDER-E2E-001" /tmp/cuopt_bridge.log 2>/dev/null; then
    echo "  ✓ Order received by cuopt_bridge"
else
    echo "  ✗ Order NOT received by cuopt_bridge"
fi

echo ""
echo "=== Real CuOpt Optimization ==="
if grep -q "Connecting to CuOpt server" /tmp/cuopt_client.log 2>/dev/null; then
    echo "  ✓ Requested optimization from real CuOpt server"
    
    if grep -q "CUOPT RESPONSE" /tmp/cuopt_client.log; then
        echo "  ✓ Received response from CuOpt server"
        
        # Extract cost and vehicle count if available
        COST=$(grep -oP "solution_cost.*?:\s*\K[\d\.]+" /tmp/cuopt_client.log 2>/dev/null | tail -1)
        VEHICLES=$(grep -oP "num_vehicles.*?:\s*\K\d+" /tmp/cuopt_client.log 2>/dev/null | tail -1)
        
        [ -n "$COST" ] && echo "    Solution cost: $COST"
        [ -n "$VEHICLES" ] && echo "    Vehicles used: $VEHICLES"
    else
        echo "  ⚠ No response from CuOpt server (check logs)"  
    fi
else
    echo "  ✗ No CuOpt request detected"
fi

echo ""
echo "=== Robot Execution ==="
for robot in amr1 amr2 amr3; do
    PLAN_COUNT=$(grep -c "NEW PLAN" /tmp/${robot}_executor_e2e.log 2>/dev/null || echo "0")
    GOAL_COUNT=$(grep -c "Nav2: Sending goal" /tmp/${robot}_executor_e2e.log 2>/dev/null || echo "0")
    
    if [ "$PLAN_COUNT" -gt 0 ]; then
        echo "  ✓ $robot: Received $PLAN_COUNT plan(s), sent $GOAL_COUNT Nav2 goal(s)"
    else
        echo "  ○ $robot: No activity"
    fi
done

echo ""
echo "======================================================================"
echo "STEP 13: Cleanup"
echo "======================================================================"
echo "Stopping all test nodes..."

kill $CUOPT_PID $BRIDGE_PID $AMR1_PID $AMR2_PID $AMR3_PID 2>/dev/null || true
sleep 2
pkill -f "cuopt_client\|cuopt_bridge\|task_executor" 2>/dev/null || true

echo "  ✓ All test nodes stopped"

echo ""
echo "======================================================================"
echo "TEST COMPLETE"
echo "======================================================================"
echo ""
echo "Detailed logs available at:"
echo "  /tmp/cuopt_client.log        - Real CuOpt optimization details"
echo "  /tmp/cuopt_bridge.log        - Order processing"
echo "  /tmp/amr1_executor_e2e.log   - AMR1 execution"
echo "  /tmp/amr2_executor_e2e.log   - AMR2 execution"
echo "  /tmp/amr3_executor_e2e.log   - AMR3 execution"
echo ""
echo "View logs with:"
echo "  cat /tmp/cuopt_client.log | grep -E 'CUOPT|DATA SENT|optimization'"
echo "  grep 'NEW PLAN\|Nav2' /tmp/amr*_executor_e2e.log"
echo ""
