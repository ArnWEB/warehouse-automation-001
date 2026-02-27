# Warehouse Fleet Automation System

## Architecture Overview

This warehouse automation system uses ROS 2 Humble with NVIDIA CuOpt for route optimization. The system supports two parallel flows for task generation and processing.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        WAREHOUSE FLEET SYSTEM                              │
└─────────────────────────────────────────────────────────────────────────────┘

                              TWO TASK FLOWS
══════════════════════════════════════════════════════════════════════════════

  FLOW 1: ORDER-BASED                     FLOW 2: FLEET-BASED
  ─────────────────────                    ────────────────────
                                           
  ┌──────────────┐                         ┌──────────────────┐     
  │ order_       │                         │ fleet_task_      │     
  │ generator    │                         │ generator        │     
  └──────┬───────┘                         └────────┬─────────┘     
         │                                        │                  
         ▼                                        ▼                  
  ┌──────────────┐                         ┌──────────────────┐     
  │ /orders      │                         │ /fleet/tasks     │     
  │ (String)     │                         │ (Int32MultiArray)│     
  └──────┬───────┘                         └────────┬─────────┘     
         │                                        │                  
         ▼                                        ▼                  
  ┌──────────────┐                         ┌──────────────────┐     
  │ cuopt_bridge │                         │ fleet_manager    │     
  │              │                         │                  │     
  │ - Mock plan  │                         │ - Collect tasks  │     
  │ - Trigger    │                         │ - Trigger cuOpt │     
  └──────┬───────┘                         └────────┬─────────┘     
         │                                        │                  
         ▼                                        ▼                  
  ┌──────────────┐      ┌──────────────────────────────────────────┐     
  │ /cuopt/trigger    │──▶│           cuopt_client                  │     
  │ (Int32MultiArray) │   │                                          │     
  └──────────────┘      │  - Subscribes to /cuopt/trigger           │     
                        │  - Subscribes to /fleet/robot_states      │     
  ┌──────────────┐      │  - Calls NVIDIA CuOpt API                 │     
  │ /cuopt/plan │      │  - Publishes optimized plan                │     
  │ (String)    │      └──────────────────────┬─────────────────────┘     
  └──────────────┘                             │                           
                                              ▼                           
                        ┌──────────────────────────────────────────────┐   
                        │           /fleet/cuopt_plan                │   
                        │           (String - JSON)                   │   
                        └──────────────────────┬───────────────────────┘   
                                                │                           
                                                ▼                           
                        ┌──────────────────────────────────────────────┐   
                        │           TASK EXECUTION                    │   
                        │  ┌──────────┐ ┌──────────┐ ┌──────────┐  │   
                        │  │ Executor  │ │ Executor  │ │ Executor  │  │   
                        │  │  AMR1    │ │  AMR2    │ │  AMR3    │  │   
                        │  └──────────┘ └──────────┘ └──────────┘  │   
                        └──────────────────────────────────────────────┘   
```

## ROS Topics

### Core Topics

| Topic | Type | Purpose | Publishers | Subscribers |
|-------|------|---------|------------|-------------|
| `/orders` | String (JSON) | Warehouse orders from order system | order_generator | cuopt_bridge |
| `/fleet/tasks` | Int32MultiArray | Task IDs for fleet | fleet_task_generator | fleet_manager |
| `/cuopt/trigger` | Int32MultiArray | Waypoint IDs for CuOpt optimization | cuopt_bridge, fleet_manager | cuopt_client |
| `/cuopt/plan` | String (JSON) | Mock optimization plan | cuopt_bridge | - |
| `/fleet/robot_states` | String (JSON) | Robot positions and status | cuopt_bridge, robot_state_monitor | cuopt_client, fleet_manager |
| `/fleet/cuopt_plan` | String (JSON) | Optimized assignment from CuOpt | cuopt_client | fleet_manager, task_executor |
| `/fleet/status` | String (JSON) | Fleet-wide status | cuopt_bridge, fleet_manager | fleet_dashboard |
| `/fleet/viz` | String (JSON) | Visualization data | fleet_visualization | - |

### Robot-Specific Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/amr{N}/odom` | Odometry | Robot position |
| `/amr{N}/status` | Int32 | 0=idle, >0=busy |
| `/amr{N}/executor_debug` | String (JSON) | Execution state |
| `/amr{N}/task` | String (JSON) | Task assignment |

## Warehouse Locations

The system uses real coordinates from Isaac Sim warehouse (90 waypoints). Key locations:

```
                        Y (meters)
                        │
    61.49 ─────────────────────────────────────────────────────────
           │                                                        │
           │     0: inbound_dock (17.98, 61.49)                   │
           │                                                        │
    58.46 ─┼────────────────────────────────────────────────────────
           │     2: palletizer (20.26, 58.46)    4: quality (17.98,58.46)
           │                                                        │
    53.34 ─┼──────────────────  10: asrs_input (12.25, 53.34)─────
           │                                                        │
    47.70 ─┼────────────────────────────────────────────────────────
           │     15: asrs_output (17.98, 47.70)                    │
           │                                                        │
     8.79 ─┼────────────────────────────────────────────────────────
           │     75: staging (17.98, 8.79)    77: outbound (30.2, 8.79)
           │                                                        │
     4.16 ─┼────────────────────────────────────────────────────────
                 80: charging_station (17.98, 4.16)
                        │
     0 +──────────────────────────────────────────────────────── X
           1.48                                              34.42
```

### Location to Waypoint Mapping

| Location Name | Waypoint ID | X | Y | Zone |
|--------------|-------------|---|---|------|
| inbound_dock | 0 | 17.98 | 61.49 | Z1 |
| palletizer | 2 | 20.26 | 58.46 | Z1 |
| quality_check | 4 | 17.98 | 58.46 | Z2 |
| asrs_input | 10 | 12.25 | 53.34 | Z2 |
| asrs_storage | 5 | 12.25 | 58.46 | Z3 |
| asrs_output | 15 | 17.98 | 47.70 | Z4 |
| staging_area | 75 | 17.98 | 8.79 | Z4 |
| outbound_dock | 77 | 30.20 | 8.79 | Z5 |
| charging_station | 80 | 17.98 | 4.16 | Z0 |

## Full Waypoint Graph (90 Waypoints)

The warehouse has 90 waypoints defined in Isaac Sim. See `solve_cuopt.py` or `/tmp/cuopt_waypoint_graph.json` for the complete waypoint graph with edges and weights.

## Packages

### order_system
- `order_generator.py` - Generates warehouse orders periodically
- `order_listener.py` - Listens to orders
- `order_publisher.py` - Alternative order publisher

### cuopt_bridge
- `cuopt_bridge.py` - Bridge node that:
  - Subscribes to `/orders`
  - Publishes mock optimization plans to `/cuopt/plan`
  - Publishes waypoint IDs to `/cuopt/trigger`
  - Publishes robot states to `/fleet/robot_states`
- `cuopt_client.py` - CuOpt client that:
  - Subscribes to `/cuopt/trigger` and `/fleet/robot_states`
  - Calls NVIDIA CuOpt API (or mock fallback)
  - Publishes optimized plans to `/fleet/cuopt_plan`

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

### warehouse_msgs
Custom message definitions (if used)

## Running the System

### Option 1: Complete System with Order Flow
```bash
# Build
colcon build --packages-select order_system cuopt_bridge

# Terminal 1: Run order generator and cuopt bridge
ros2 run order_system order_generator
ros2 run cuopt_bridge cuopt_bridge

# Terminal 2: Run cuopt client
ros2 run cuopt_bridge cuopt_client
```

### Option 2: Fleet-Based Flow
```bash
# Build
colcon build --packages-select orchestrator cuopt_bridge

# Terminal 1: Run fleet task generator and manager
ros2 run orchestrator fleet_task_generator
ros2 run orchestrator fleet_manager

# Terminal 2: Run cuopt client
ros2 run cuopt_bridge cuopt_client
```

### Option 3: Full System with Gazebo
```bash
# Build all
colcon build

# Terminal 1: Start Gazebo
ros2 launch order_system complete_warehouse.launch.py

# Terminal 2: Run fleet system
ros2 launch orchestrator production_fleet.launch.py
```

## Data Flow Examples

### Order-Based Flow
```
1. order_generator publishes:
   /orders = {"order_id": "ORD-001", "items": ["item1"], "quantities": [1], "priority": 3}

2. cuopt_bridge receives order:
   - Creates mock plan with tasks
   - Publishes to /cuopt/plan
   - Extracts waypoint IDs: [2, 4, 10] (palletizer→quality→asrs_input)
   - Publishes to /cuopt/trigger = [2, 4, 10]
   - Publishes robot positions to /fleet/robot_states

3. cuopt_client receives trigger + robot states:
   - Calls CuOpt API with waypoint graph
   - Returns optimized assignments
   - Publishes to /fleet/cuopt_plan = {
       "plan_id": 1,
       "assignments": {
         "amr1": {"tasks": [2, 4]},
         "amr2": {"tasks": [10]}
       },
       "total_cost": 322.70
     }
```

### Fleet-Based Flow
```
1. fleet_task_generator publishes:
   /fleet/tasks = [6, 7, 8]

2. fleet_manager receives tasks:
   - Collects robot states from /fleet/robot_states
   - Publishes waypoint IDs to /cuopt/trigger

3. cuopt_client optimizes and publishes to /fleet/cuopt_plan
```

## CuOpt Integration

### Real CuOpt API
The system integrates with NVIDIA CuOpt server:
- Server: `43.201.55.122:5000`
- Uses waypoint graph from Isaac Sim
- Supports fallback to mock solver

### Waypoint Graph
- Loaded from `/tmp/cuopt_waypoint_graph.json` (generated from Isaac Sim)
- Fallback to static graph in `cuopt_client.py`
- 90 waypoints with edges and weights

### Coordinate System
- Uses real Isaac Sim coordinates (1.48 - 61.49 meters)
- Both cuopt_bridge and cuopt_client must use same coordinate system
- Waypoint mapping ensures correct position matching

## Configuration

### Robot Fleet
Defined in `cuopt_bridge.py`:
```python
ROBOTS = {
    "amr1": {"type": "forklift", "capacity_kg": 1000, "current_loc": "charging_station"},
    "amr2": {"type": "forklift", "capacity_kg": 1000, "current_loc": "staging_area"},
}
```

### CuOpt Server
Defined in `cuopt_client.py`:
```python
CUOPT_SERVER_IP = "43.201.55.122"
CUOPT_SERVER_PORT = 5000
```

## Monitoring Commands

```bash
# See all topics
ros2 topic list

# Watch orders
ros2 topic echo /orders

# Watch CuOpt trigger
ros2 topic echo /cuopt/trigger

# Watch robot states
ros2 topic echo /fleet/robot_states

# Watch optimized plans
ros2 topic echo /fleet/cuopt_plan

# Watch fleet status
ros2 topic echo /fleet/status

# Check running nodes
ros2 node list
```

## Current Status

### Working
- ✅ Order generation (every 5 seconds)
- ✅ CuOpt bridge (mock + real coordinates)
- ✅ CuOpt client (real API integration)
- ✅ Task optimization with 90-waypoint graph
- ✅ Robot state publishing with real coordinates

### Known Issues
- ⚠️ Some PEP257/flake8 linting warnings in cuopt_bridge.py
- ⚠️ Robot movement in Gazebo requires ros2_control setup

## File Structure

```
warehouse-automation/
├── ARCHITECTURE.md              # This file
├── AGENTS.md                   # Agent instructions
├── solve_cuopt.py               # CuOpt reference implementation
│
├── order_system/
│   ├── order_system/
│   │   ├── order_generator.py
│   │   └── order_listener.py
│   └── launch/
│
├── cuopt_bridge/
│   ├── cuopt_bridge/
│   │   ├── cuopt_bridge.py      # Bridge with real coordinates
│   │   └── cuopt_client.py      # CuOpt API client
│   └── launch/
│
├── orchestrator/
│   ├── orchestrator/
│   │   ├── fleet_manager.py
│   │   ├── fleet_task_generator.py
│   │   ├── robot_state_monitor.py
│   │   ├── task_executor.py
│   │   ├── fleet_dashboard.py
│   │   └── fleet_visualization.py
│   └── launch/
│
├── amr_description/
│   ├── urdf/
│   ├── worlds/
│   └── scripts/
│
└── warehouse_msgs/
    └── msg/
```

## Troubleshooting

### CuOpt Connection Failed
```
Failed to connect to CuOpt server at 43.201.55.122:5000
```
**Solution**: System falls back to mock solver automatically.

### Wrong Waypoint Mapping
If robots map to wrong waypoints:
1. Check that cuopt_bridge.py uses real Isaac Sim coordinates
2. Verify WAREHOUSE_LOCATIONS match in both files
3. Check /fleet/robot_states has correct x,y values

### Topics Not Publishing
```bash
# Verify nodes are running
ros2 node list

# Check topic hz
ros2 topic hz /cuopt/trigger

# Check for errors
ros2 node info /cuopt_bridge
```
