# Warehouse Fleet Automation System

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        WAREHOUSE FLEET SYSTEM                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│  1. TASK GENERATION                                                        │
│  ┌──────────────────┐     ┌──────────────────┐                           │
│  │ Fleet Task       │────▶│ /fleet/tasks     │                           │
│  │ Generator       │     │ (Int32MultiArray)│                           │
│  │ (every 5s)     │     └────────┬─────────┘                           │
│  └──────────────────┘              │                                      │
└────────────────────────────────────┼───────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  2. FLEET MANAGEMENT                                                      │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐  │
│  │ Fleet Manager    │────▶│ Robot State     │────▶│ CuOpt Client    │  │
│  │                  │     │ Monitor         │     │                  │  │
│  │ - Collects tasks │     │                 │     │ - Optimization  │  │
│  │ - Triggers cuOpt│     │ - Robot pos     │     │ - Robot assign  │  │
│  └──────────────────┘     └──────────────────┘     └────────┬─────────┘  │
│                                                              │             │
└──────────────────────────────────────────────────────────────┼─────────────┘
                                                               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  3. OPTIMIZATION (cuOpt)                                                │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  /fleet/cuopt_plan                                               │   │
│  │  {                                                               │   │
│  │    "plan_id": 1,                                                 │   │
│  │    "assignments": {                                              │   │
│  │      "amr1": {"tasks": [1, 6]},                                │   │
│  │      "amr2": {"tasks": [3]},                                    │   │
│  │      "amr3": {"tasks": [7]}                                     │   │
│  │    }                                                             │   │
│  │  }                                                               │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                     │
                    ┌─────────────────┼─────────────────┐
                    ▼                 ▼                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  4. TASK EXECUTION (Per Robot)                                            │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐        │
│  │ Executor AMR1    │  │ Executor AMR2    │  │ Executor AMR3    │        │
│  │                  │  │                  │  │                  │        │
│  │ - Subscribe plan │  │ - Subscribe plan │  │ - Subscribe plan │        │
│  │ - Execute tasks │  │ - Execute tasks │  │ - Execute tasks │        │
│  │ - Publish debug │  │ - Publish debug │  │ - Publish debug │        │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  5. VISUALIZATION/OUTPUT                                                  │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐        │
│  │ Fleet Dashboard   │  │ Fleet Viz        │  │ Gazebo (if avail)│        │
│  │                  │  │                  │  │                  │        │
│  │ - Shows flow    │  │ - Graph nodes   │  │ - Robot movement │        │
│  │ - Robot status  │  │ - Edges/routes  │  │ - cmd_vel       │        │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
```

## ROS Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/fleet/tasks` | Int32MultiArray | New tasks to process |
| `/fleet/robot_states` | String (JSON) | Robot positions/status |
| `/fleet/cuopt_plan` | String (JSON) | Optimized assignment |
| `/fleet/viz` | String (JSON) | Graph visualization |
| `/fleet/status` | String (JSON) | Fleet status |
| `/amr{N}/odom` | Odometry | Robot position |
| `/amr{N}/status` | Int32 | 0=idle, >0=busy |
| `/amr{N}/executor_debug` | String (JSON) | Execution state |

## Warehouse Locations

```
           Y
           │
    4       │       5
  asrs_store│    asrs_out
           │
    1       │       3
 palletizer │    asrs_in
           │
-----------+-------------------- X
           │
    2       │       6       7
  quality   │    staging  outbound
           │
           0
       charging
```

### Location Coordinates

| ID | Name | X | Y |
|----|------|---|---|
| 0 | charging | -5.0 | 0.0 |
| 1 | palletizer | -8.0 | 3.0 |
| 2 | quality_check | -4.0 | 3.0 |
| 3 | asrs_in | 8.0 | 3.0 |
| 4 | asrs_store | 8.0 | 5.0 |
| 5 | asrs_out | 8.0 | 7.0 |
| 6 | staging | 0.0 | -5.0 |
| 7 | outbound | 5.0 | -5.0 |

## Packages

### order_system
- `order_generator.py` - Generates warehouse orders
- `order_listener.py` - Listens to orders

### cuopt_bridge
- `cuopt_bridge.py` - Original mock cuOpt
- `cuopt_client.py` - New cuOpt client (supports real API)

### orchestrator
- `fleet_manager.py` - Central fleet orchestration
- `fleet_task_generator.py` - Generates tasks for fleet
- `robot_state_monitor.py` - Monitors robot states
- `task_executor.py` - Executes plans per robot
- `fleet_dashboard.py` - Visualizes execution flow
- `fleet_visualization.py` - Graph visualization

### amr_description
- `amr_robot.urdf` - Robot model for Gazebo
- `amr_robot_diffdrive.urdf` - Robot with ros2_control
- `warehouse.world` - Gazebo world
- Various launch files

## Running the System

### Option 1: Single Command (Recommended)
```bash
# Build
colcon build --packages-select order_system orchestrator cuopt_bridge amr_description

# Run everything
ros2 launch order_system complete_warehouse_fleet.launch.py
```

### Option 2: Two Terminals
```bash
# Terminal 1: Gazebo + robots
ros2 launch order_system complete_warehouse.launch.py

# Terminal 2: Fleet system
ros2 launch orchestrator production_fleet.launch.py
```

## Monitoring Commands

```bash
# See all topics
ros2 topic list

# Watch tasks being generated
ros2 topic echo /fleet/tasks

# Watch CuOpt optimization plans
ros2 topic echo /fleet/cuopt_plan

# Watch robot execution
ros2 topic echo /amr1/executor_debug
ros2 topic echo /amr2/executor_debug
ros2 topic echo /amr3/executor_debug

# Check running nodes
ros2 node list
```

## Launch Files

### complete_warehouse_fleet.launch.py
Complete system with Gazebo and fleet management. Runs:
1. Gazebo server + client
2. Spawns 3 AMR robots
3. Fleet task generator
4. Fleet manager
5. CuOpt client
6. 3 Task executors (one per robot)
7. Fleet dashboard

### production_fleet.launch.py
Fleet management only (requires Gazebo already running):
- Fleet task generator
- Fleet manager
- Robot state monitor
- CuOpt client
- Task executors
- Fleet dashboard

### complete_warehouse.launch.py
Gazebo + robots only (no fleet system):
- Gazebo server with ROS plugins
- Gazebo client (GUI)
- Spawns 3 AMR robots
- Order system nodes

## Data Flow Example

```
1. Task Generator publishes: [6]
   └─> /fleet/tasks = [6]

2. Fleet Manager receives tasks
   └─> Triggers CuOpt optimization

3. CuOpt Client optimizes:
   └─> /fleet/cuopt_plan = {
        "plan_id": 1,
        "assignments": {
          "amr1": {"tasks": [6]},
          "amr2": {"tasks": []},
          "amr3": {"tasks": []}
        }
      }

4. Executors receive plan:
   └─> AMR1: moving to staging (location 6)
   └─> AMR2: idle
   └─> AMR3: idle

5. Dashboard shows:
   ┌─────────────────────────────────────┐
   │ 🚗 WAREHOUSE FLEET STATUS          │
   │ ─────────────────────────────────── │
   │ 📋 Pending Tasks: [6]               │
   │                                     │
   │ 🤖 amr1: moving to staging         │
   │ 🤖 amr2: idle                      │
   │ 🤖 amr3: idle                      │
   │                                     │
   │ 📍 Warehouse Locations:             │
   │   ⏳ 0: charging (-5.0,  0.0)      │
   │   ⏳ 1: palletizer (-8.0,  3.0)     │
   │   📌 6: staging ( 0.0, -5.0)       │
   └─────────────────────────────────────┘
```

## Current Status

### Working
- ✅ Task generation (every 5 seconds)
- ✅ Fleet Manager orchestration
- ✅ CuOpt optimization (mock solver)
- ✅ Task executors receive and process plans
- ✅ Dashboard shows execution flow
- ✅ Robots visible in Gazebo

### Not Working (Requires Additional Setup)
- ❌ Robot movement in Gazebo (needs ros2_control)
- ❌ set_model_state service unavailable

### To Enable Robot Movement

Install and configure gazebo_ros2_control:

```bash
# Install
sudo apt install ros-humble-gazebo-ros2-control

# Use diffdrive URDF
# Update launch to use: amr_robot_diffdrive.urdf
```

## File Structure

```
warehouse-automation/
├── order_system/
│   ├── order_system/
│   │   ├── order_generator.py
│   │   └── order_listener.py
│   ├── launch/
│   │   ├── complete_warehouse.launch.py
│   │   └── complete_warehouse_fleet.launch.py  ⬅ NEW
│   └── setup.py
│
├── cuopt_bridge/
│   ├── cuopt_bridge/
│   │   ├── cuopt_bridge.py
│   │   └── cuopt_client.py  ⬅ NEW
│   └── setup.py
│
├── orchestrator/
│   ├── orchestrator/
│   │   ├── orchestrator.py
│   │   ├── amr_robot.py
│   │   ├── fleet_manager.py  ⬅ NEW
│   │   ├── fleet_task_generator.py  ⬅ NEW
│   │   ├── robot_state_monitor.py  ⬅ NEW
│   │   ├── task_executor.py  ⬅ NEW
│   │   ├── fleet_dashboard.py  ⬅ NEW
│   │   └── fleet_visualization.py  ⬅ NEW
│   ├── launch/
│   │   └── production_fleet.launch.py  ⬅ NEW
│   └── setup.py
│
├── amr_description/
│   ├── urdf/
│   │   ├── amr_robot.urdf
│   │   └── amr_robot_diffdrive.urdf  ⬅ NEW
│   ├── worlds/
│   │   └── warehouse.world
│   ├── config/
│   │   └── amr_controllers.yaml  ⬅ NEW
│   └── launch/
│
└── warehouse_msgs/
    └── msg/
        ├── Task.msg
        ├── RobotState.msg
        ├── FleetPlan.msg
        └── Order.msg
```
