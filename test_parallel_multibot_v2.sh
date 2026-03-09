#!/bin/bash

# Multi-Robot Parallel Execution Test (Improved)
# All 3 robots execute their assigned tasks concurrently
# Uses multiple plan publishes to ensure all robots receive it

cd /home/ubuntu/warehouse-automation-001
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "======================================================================"
echo "      PARALLEL MULTI-ROBOT WAREHOUSE AUTOMATION TEST (V2)"
echo "======================================================================"

echo ""
echo "STEP 1:  Cleanup"
echo "======================================================================"
pkill -f task_executor 2>/dev/null || true
sleep 2
pkill -f "ros2 run orchestrator" 2>/dev/null || true
sleep 1

echo ""
echo "STEP 2: Verify Infrastructure"
echo "======================================================================"
echo "✓ Nav2 action servers:"
ros2 action list | grep navigate_to_pose

echo ""
echo "✓ Isaac Sim odometry topics:"
ros2 topic list | grep chassis/odom

echo ""
echo "STEP 3: Launch 3 Task Executors (Nav2 mode)"
echo "======================================================================"

echo "  Starting amr1..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr1 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr1_executor.log 2>&1 &
PID_AMR1=$!
sleep 2

echo "  Starting amr2..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr2 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr2_executor.log 2>&1 &
PID_AMR2=$!
sleep 2

echo "  Starting amr3..."
ros2 run orchestrator task_executor --ros-args \
  -p robot_id:=amr3 \
  -p use_nav2:=true \
  -p use_global_namespace:=false \
  > /tmp/amr3_executor.log 2>&1 &
PID_AMR3=$!
sleep 2

echo "✓ All 3 executors running"

echo ""
echo "STEP 4: Wait for Executor Initialization"
echo "======================================================================"
echo "Waiting 5 seconds for /fleet/cuopt_plan subscribers..."
sleep 5

echo ""
echo "STEP 5: PUBLISH CuOpt PLAN (Multiple Times for Reliability)"
echo "======================================================================"
echo "Plan:"
echo "  amr1 → [10, 20, 30]  (Zone A: waypoint_10, waypoint_20, waypoint_30)"
echo "  amr2 → [40, 50, 60]  (Zone B: waypoint_40, waypoint_50, waypoint_60)" 
echo "  amr3 → [70, 80, 90]  (Zone C: waypoint_70, waypoint_80, waypoint_90)"
echo ""
echo "Publishing plan 5 times to ensure all robots receive it..."

# Publish multiple times to ensure delivery
for i in {1..5}; do
  ros2 topic pub /fleet/cuopt_plan std_msgs/String \
    "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10, 20, 30]}, \"amr2\": {\"tasks\": [40, 50, 60]}, \"amr3\": {\"tasks\": [70, 80, 90]}}}'" \
    --once > /dev/null 2>&1
  echo "  ✓ Publish $i/5"
  sleep 0.3
done

echo ""
echo "STEP 6: Real-Time Parallel Execution Monitoring (60 seconds)"
echo "======================================================================"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper function to get robot busy status
get_robot_busy() {
  local robot_id=$1
  timeout 0.5 ros2 topic echo /${robot_id}/robot_state --once 2>/dev/null | grep -o '"current_task": [^,]*' | awk -F: '{print $2}' | tr -d ' "' || echo "?"
}

echo "Monitoring: Format: [seconds] amr1:task | amr2:task | amr3:task"
echo ""

for i in {1..60}; do
  task1=$(get_robot_busy amr1)
  task2=$(get_robot_busy amr2)
  task3=$(get_robot_busy amr3)
  
  # Format output with task status
  if [[ "$task1" == "?" ]]; then
    t1_display="N/A"
  elif [[ "$task1" == "-1" ]]; then
    t1_display="idle"
  else
    t1_display="T${task1}"
  fi
  
  if [[ "$task2" == "?" ]]; then
    t2_display="N/A"
  elif [[ "$task2" == "-1" ]]; then
    t2_display="idle"
  else
    t2_display="T${task2}"
  fi
  
  if [[ "$task3" == "?" ]]; then
    t3_display="N/A"
  elif [[ "$task3" == "-1" ]]; then
    t3_display="idle"
  else
    t3_display="T${task3}"
  fi
  
  printf "[%3d sec] amr1:%5s | amr2:%5s | amr3:%5s\r" "$i" "$t1_display" "$t2_display" "$t3_display"
  sleep 1
done
echo ""
echo ""

echo "STEP 7: Final Status Report"
echo "======================================================================"

echo ""
echo "=== AMR1 Final State ==="
timeout 2 ros2 topic echo /amr1/robot_state --once 2>/dev/null || echo "No state received"

echo ""
echo "=== AMR2 Final State ==="
timeout 2 ros2 topic echo /amr2/robot_state --once 2>/dev/null || echo "No state received"

echo ""
echo "=== AMR3 Final State ==="
timeout 2 ros2 topic echo /amr3/robot_state --once 2>/dev/null || echo "No state received"

echo ""
echo "STEP 8: Execution Summary from Logs"
echo "======================================================================"

echo ""
echo "=== AMR1 Execution Log (key events) ==="
grep -E "NEW PLAN|EXECUTING|Nav2:" /tmp/amr1_executor.log 2>/dev/null | head -8 || echo "No execution events logged for amr1"

echo ""
echo "=== AMR2 Execution Log (key events) ==="
grep -E "NEW PLAN|EXECUTING|Nav2:" /tmp/amr2_executor.log 2>/dev/null | head -8 || echo "No execution events logged for amr2"

echo ""
echo "=== AMR3 Execution Log (key events) ==="
grep -E "NEW PLAN|EXECUTING|Nav2:" /tmp/amr3_executor.log 2>/dev/null | head -8 || echo "No execution events logged for amr3"

echo ""
echo "STEP 9: Verification Metrics"
echo "======================================================================"

echo ""
echo "Plan Reception:"
if grep -q "NEW PLAN" /tmp/amr1_executor.log 2>/dev/null; then
  echo "  ✓ amr1 RECEIVED and PROCESSED plan"
  grep "NEW PLAN" /tmp/amr1_executor.log | head -1 | sed 's/^/    /'
else
  echo "  ✗ amr1 DID NOT receive plan"
fi

if grep -q "NEW PLAN" /tmp/amr2_executor.log 2>/dev/null; then
  echo "  ✓ amr2 RECEIVED and PROCESSED plan"
  grep "NEW PLAN" /tmp/amr2_executor.log | head -1 | sed 's/^/    /'
else
  echo "  ✗ amr2 DID NOT receive plan"
fi

if grep -q "NEW PLAN" /tmp/amr3_executor.log 2>/dev/null; then
  echo "  ✓ amr3 RECEIVED and PROCESSED plan"
  grep "NEW PLAN" /tmp/amr3_executor.log | head -1 | sed 's/^/    /'
else
  echo "  ✗ amr3 DID NOT receive plan"
fi

echo ""
echo "Navigation Goals Sent to Nav2:"
if grep -q "Nav2: Sending goal" /tmp/amr1_executor.log 2>/dev/null; then
  echo "  ✓ amr1 sent goals to Nav2"
  grep "Nav2: Sending goal" /tmp/amr1_executor.log | wc -l | xargs echo "    Number of goals:"
else
  echo "  ✗ amr1 did NOT send goals"
fi

if grep -q "Nav2: Sending goal" /tmp/amr2_executor.log 2>/dev/null; then
  echo "  ✓ amr2 sent goals to Nav2"
  grep "Nav2: Sending goal" /tmp/amr2_executor.log | wc -l | xargs echo "    Number of goals:"
else
  echo "  ✗ amr2 did NOT send goals"
fi

if grep -q "Nav2: Sending goal" /tmp/amr3_executor.log 2>/dev/null; then
  echo "  ✓ amr3 sent goals to Nav2"
  grep "Nav2: Sending goal" /tmp/amr3_executor.log | wc -l | xargs echo "    Number of goals:"
else
  echo "  ✗ amr3 did NOT send goals"
fi

echo ""
echo "STEP 10: Cleanup"
echo "======================================================================"
echo "Killing task executors..."
kill $PID_AMR1 $PID_AMR2 $PID_AMR3 2>/dev/null || true
sleep 1
pkill -f task_executor 2>/dev/null || true

echo "✓ All executors stopped"
echo ""

echo "======================================================================"
echo "TEST COMPLETED"
echo "======================================================================"
echo ""
echo "Quick Log Review Commands:"
echo "  Full logs: tail -100 /tmp/amr{1,2,3}_executor.log"
echo "  Grep plans: grep 'NEW PLAN' /tmp/amr{1,2,3}_executor.log"
echo "  Grep nav2: grep 'Nav2:' /tmp/amr{1,2,3}_executor.log"
echo ""
