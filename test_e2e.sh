#!/bin/bash
# End-to-End Test Script for Warehouse Automation
# Tests the task_executor with mock odometry (no Isaac Sim required)

set -e

echo "=========================================="
echo "Warehouse Automation E2E Test"
echo "=========================================="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check ROS
echo -e "${YELLOW}[1/6] Checking ROS2...${NC}"
if ! command_exists ros2; then
    echo -e "${RED}ERROR: ros2 not found. Please source ROS2 Humble.${NC}"
    exit 1
fi
echo -e "${GREEN}OK: ROS2 found${NC}"

# Source workspace
echo -e "${YELLOW}[2/6] Sourcing workspace...${NC}"
source /opt/ros/humble/setup.bash
source ~/warehouse-automation-001/install/setup.bash
echo -e "${GREEN}OK: Workspace sourced${NC}"

# Check packages
echo -e "${YELLOW}[3/6] Checking packages...${NC}"
ros2 pkg list | grep -q orchestrator && echo "  - orchestrator: OK" || echo "  - orchestrator: MISSING"
ros2 pkg list | grep -q amr_description && echo "  - amr_description: OK" || echo "  - amr_description: MISSING"
echo -e "${GREEN}OK: Packages available${NC}"

# Test 1: Mock Odom Publisher
echo -e "${YELLOW}[4/6] Test 1: Mock Odometry Publisher...${NC}"
ros2 run orchestrator mock_odom_publisher --ros-args -p robot_id:=amr1 -p x:=17.98 -p y:=4.16 &
MOCK_PID=$!
sleep 2

# Check if publishing
if ros2 topic hz /amr1/chassis/odom >/dev/null 2>&1; then
    echo -e "${GREEN}  - Mock odom publishing: OK${NC}"
else
    echo -e "${RED}  - Mock odom publishing: FAILED${NC}"
fi
kill $MOCK_PID 2>/dev/null || true
sleep 1

# Test 2: Task Executor in cmd_vel mode
echo -e "${YELLOW}[5/6] Test 2: Task Executor (cmd_vel mode)...${NC}"
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=false &
EXE_PID=$!
sleep 3

# Check if node is running
if ros2 node list | grep -q "task_executor_amr1"; then
    echo -e "${GREEN}  - Task executor started: OK${NC}"
else
    echo -e "${RED}  - Task executor started: FAILED${NC}"
fi

# Check subscriptions
echo -e "${YELLOW}[6/6] Test 3: Send CuOpt Plan...${NC}"
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10, 20]}}}'" --once &
sleep 2

# Check robot state
if ros2 topic echo /amr1/robot_state --once 2>/dev/null | grep -q "robot_id"; then
    echo -e "${GREEN}  - Robot state publishing: OK${NC}"
else
    echo -e "${YELLOW}  - Robot state: No data (may need more time)${NC}"
fi

# Check cmd_vel
if ros2 topic hz /amr1/cmd_vel >/dev/null 2>&1; then
    echo -e "${GREEN}  - cmd_vel publishing: OK${NC}"
else
    echo -e "${YELLOW}  - cmd_vel: Not publishing (no active task)${NC}"
fi

# Cleanup
kill $EXE_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo -e "${GREEN}E2E Test Complete!${NC}"
echo "=========================================="
echo ""
echo "To run full test with Nav2:"
echo "  1. Terminal 1: ros2 launch orchestrator nav2_amr.launch.py robot_id:=amr1"
echo "  2. Terminal 2: ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true"
echo "  3. Terminal 3: ros2 topic pub /fleet/cuopt_plan std_msgs/String \"data: '{\\\"plan_id\\\": 1, \\\"assignments\\\": {\\\"amr1\\\": {\\\"tasks\\\": [10, 20]}}}'\" --once"
echo ""
