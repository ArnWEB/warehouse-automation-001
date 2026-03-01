# Warehouse Automation System

## Overview

ROS 2 Humble warehouse automation system with:
- **CuOpt Bridge**: Task optimization and assignment
- **Order System**: Order generation and listening  
- **Orchestrator**: Task execution and robot management
- **AMR Description**: Robot URDF, Gazebo world, Isaac Sim scripts
- **Nav2 Integration**: Navigation stack for robot path planning

---

## Architecture

```
                    +------------------+
                    |   CuOpt Bridge  |
                    | (Task Optimizer)|
                    +--------+---------+
                             |
                             v
                    +------------------+
                    |  /fleet/cuopt   |
                    |     _plan       |
                    +--------+---------+
                             |
        +--------------------+--------------------+
        |                    |                    |
        v                    v                    v
+-----------+        +-----------+        +-----------+
|  AMR 1   |        |  AMR 2   |        |  AMR 3   |
|TaskExec  |        |TaskExec  |        |TaskExec  |
|-----------|        |-----------|        |-----------|
| - Nav2   |        | - Nav2   |        | - Nav2   |
| - cmd_vel|        | - cmd_vel|        | - cmd_vel|
+-----------+        +-----------+        +-----------+
        |                    |                    |
        +--------------------+--------------------+
                             |
                    +--------v---------+
                    |   Isaac Sim /   |
                    |   Gazebo Sim    |
                    +-----------------+
```

---

## Components

### 1. Task Executor (`orchestrator/task_executor.py`)

Executes CuOpt plans using either:
- **Nav2** - For map-based navigation with path planning
- **cmd_vel** - Direct velocity control (for testing/simulation)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_id` | string | `"amr1"` | Robot identifier (amr1, amr2, amr3) |
| `use_nav2` | bool | `false` | Use Nav2 navigation vs cmd_vel |
| `use_global_namespace` | bool | `true` | Use global Nav2 topics (vs namespaced) |

**Topic Subscriptions:**
- `/fleet/cuopt_plan` - CuOpt task assignments
- `/{robot_id}/chassis/odom` or `/chassis/odom` - Robot odometry

**Topic Publications:**
- `/{robot_id}/robot_state` - Current robot state (for CuOpt)
- `/{robot_id}/cmd_vel` or `/cmd_vel` - Velocity commands
- `/{robot_id}/status` - Current task status
- `/{robot_id}/executor_debug` - Debug information

**Namespace Modes:**

| Mode | use_nav2 | use_global_namespace | Topics Used |
|------|----------|---------------------|-------------|
| cmd_vel | false | N/A | `/{robot_id}/chassis/odom`, `/{robot_id}/cmd_vel` |
| Nav2 Global | true | true | `/chassis/odom`, `/cmd_vel`, `/navigate_to_pose` |
| Nav2 Namespaced | true | false | `/{robot_id}/chassis/odom`, `/{robot_id}/cmd_vel`, `/{robot_id}/navigate_to_pose` |

### 2. Mock Odometry Publisher (`orchestrator/mock_odom_publisher.py`)

Simulates robot odometry for testing without simulation.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_id` | string | `"amr1"` | Robot identifier |
| `x` | float | `17.98` | X position |
| `y` | float | `4.16` | Y position |

**Usage:**
```bash
ros2 run orchestrator mock_odom_publisher --ros-args -p robot_id:=amr1 -p x:=17.98 -p y:=4.16
```

### 3. Nav2 Launch (`orchestrator/launch/nav2_amr.launch.py`)

Launches Nav2 stack for robot navigation. **Note:** Currently requires external Nav2 (like Isaac Sim).

### 4. Warehouse Locations (`orchestrator/task_executor.py`)

Pre-defined waypoint coordinates matching the warehouse layout:

| ID | Name | X | Y |
|----|------|---|---|
| 0 | waypoint_0 | 17.98 | 61.49 |
| 10 | waypoint_10 | 12.25 | 53.34 |
| 80 | waypoint_80 | 17.98 | 4.16 |
| ... | ... | ... | ... |

---

## Installation

### Prerequisites
- ROS 2 Humble
- Nav2 packages (navigation2)
- Isaac Sim (optional, for simulation)

### Build

```bash
cd ~/warehouse-automation-001
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Testing

### Test 1: Mock Odometry + cmd_vel Mode

Tests task executor with mock odometry and direct velocity control (no Nav2).

```bash
# Terminal 1 - Mock odometry publisher
ros2 run orchestrator mock_odom_publisher --ros-args -p robot_id:=amr1

# Terminal 2 - Task executor (cmd_vel mode)
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=false

# Terminal 3 - Send CuOpt plan
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

# Terminal 4 - Monitor
ros2 topic echo /amr1/robot_state
ros2 topic echo /amr1/cmd_vel
```

**Expected Output:**
```
# Robot State
{"robot_id": "amr1", "x": 17.98, "y": 4.16, "theta": 0.0, "busy": true, "current_task": 10, ...}

# cmd_vel (robot turning toward target)
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 2.0}
```

### Test 2: Nav2 Global Namespace (Your Current Setup)

Tests task executor with Nav2 using global topics.

**Prerequisites:**
- Isaac Sim running with Nav2
- Or external Nav2 with global namespace

```bash
# Make sure Nav2 is running and activated
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate

# Terminal 1 - Task executor with Nav2
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=true

# Terminal 2 - Send CuOpt plan
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10]}}}'" --once

# Terminal 3 - Monitor
ros2 topic echo /amr1/robot_state
ros2 topic echo /cmd_vel
```

**Expected Output:**
```
# Log
Nav2 action client created: /navigate_to_pose
Nav2 server available
Mode: Nav2 (global)
Nav2: Sending goal (12.25, 53.34)
Nav2 goal accepted
```

### Test 3: Multi-Robot with Namespaced Nav2

Tests multiple robots with namespaced Nav2 topics.

**Prerequisites:**
- Multiple Nav2 instances running with robot namespaces
- Or Isaac Sim with multi-robot setup

```bash
# Terminal 1 - AMR1
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=false

# Terminal 2 - AMR2
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr2 -p use_nav2:=true -p use_global_namespace:=false

# Terminal 3 - Send multi-robot plan
ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{\"plan_id\": 1, \"assignments\": {\"amr1\": {\"tasks\": [10, 20]}, \"amr2\": {\"tasks\": [5, 15]}}}'" --once
```

### Test 4: Using cuopt_bridge

Tests the full CuOpt integration.

```bash
# Terminal 1 - Start cuopt_bridge
ros2 run cuopt_bridge cuopt_bridge

# Terminal 2 - Start task executors
ros2 run orchestrator task_executor --ros-args -p robot_id:=amr1 -p use_nav2:=true -p use_global_namespace:=true

# Terminal 3 - Generate orders
ros2 run order_system order_generator
```

---

## Topic Reference

### CuOpt Plan Format

```json
{
  "plan_id": 1,
  "assignments": {
    "amr1": {
      "tasks": [10, 20, 30]
    },
    "amr2": {
      "tasks": [5, 15]
    }
  }
}
```

### Robot State Format

```json
{
  "robot_id": "amr1",
  "x": 17.98,
  "y": 4.16,
  "theta": 0.0,
  "busy": true,
  "current_task": 10,
  "target_waypoint": 30,
  "progress": 0.33,
  "plan_id": 1
}
```

---

## File Structure

```
warehouse-automation-001/
├── AGENTS.md                    # Development guidelines
├── orchestrator/
│   ├── orchestrator/
│   │   ├── task_executor.py    # Main task execution node
│   │   ├── mock_odom_publisher.py  # Mock odometry for testing
│   │   └── robot_state_monitor.py   # Robot state monitoring
│   ├── launch/
│   │   └── nav2_amr.launch.py # Nav2 launch file
│   ├── params/
│   │   ├── nav2_gridfree.yaml  # Grid-free Nav2 params
│   │   └── nav2_isaacsim.yaml # Isaac Sim Nav2 params
│   └── setup.py
├── amr_description/
│   ├── maps/
│   │   ├── iw_hub_warehouse_navigation.yaml
│   │   └── iw_hub_warehouse_navigation.png
│   ├── gazebo/
│   │   └── warehouse.world     # Gazebo world
│   └── urdf/
│       └── amr_robot.urdf     # Robot description
├── cuopt_bridge/
│   └── cuopt_bridge/          # CuOpt integration
├── order_system/
│   └── order_system/           # Order generation
└── warehouse_msgs/
    └── warehouse_msgs/         # Custom messages
```

---

## Troubleshooting

### Nav2 Goal Rejected

**Cause:** Nav2 nodes not activated

**Solution:**
```bash
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /amcl activate
```

### No odometry data

**Cause:** Not subscribed to correct topic

**Solution:** Check which odometry topic is being published:
```bash
ros2 topic list | grep odom
ros2 topic hz /chassis/odom
ros2 topic hz /amr1/chassis/odom
```

### Task executor not receiving plans

**Cause:** Topic name mismatch

**Solution:** Verify subscription:
```bash
ros2 topic echo /fleet/cuopt_plan
ros2 topic info /fleet/cuopt_plan
```

---

## Notes

- Current Nav2 implementation requires external Nav2 (Isaac Sim or manual launch)
- The `nav2_amr.launch.py` in this repo needs additional packages (`nav2_velocity_smoother`)
- For production, use Isaac Sim's Nav2 or set up namespaced Nav2 for multi-robot
- Robot positions default to warehouse waypoints defined in `WAREHOUSE_LOCATIONS`
