# Isaac Sim Python API Documentation

A comprehensive guide for developers to get started with creating scenes in NVIDIA Isaac Sim using Python.

---

## Table of Contents

1. [Installation Directory Structure](#installation-directory-structure)
2. [Key Python Modules](#key-python-modules)
3. [Running Python Scripts in Isaac Sim](#running-python-scripts-in-isaac-sim)
4. [Core APIs for Scene Creation](#core-apis-for-scene-creation)
5. [Sample Scripts](#sample-scripts)
6. [Extension Data Paths](#extension-data-paths)
7. [ROS2 Bridge Integration](#ros2-bridge-integration)
8. [Troubleshooting](#troubleshooting)

---

## Installation Directory Structure

The Isaac Sim installation is located at:
```
/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/
```

### Key Directories

| Directory | Description |
|-----------|-------------|
| `isaac-sim.sh` | Main launch script |
| `kit/` | Kit runtime (Isaac Sim core) |
| `exts/` | Core extensions |
| `extscache/` | Cached extensions (pre-built) |
| `docs/py/api/` | Python API documentation |
| `data/` | Assets and data files |

### Extension Cache Structure

Extensions are stored in `extscache/` with the naming pattern:
```
<extension-name>-<version>+<build>.lx64.r.cp311/
```

Example:
```
omni.cuopt.examples-1.3.0+107.3.2/
omni.physx-107.3.26+107.3.3.lx64.r.cp311.u353/
isaacsim.ros2.bridge-4.12.4/
```

---

## Key Python Modules

### Core Modules (Always Available)

```python
import omni.ext          # Extension management
import omni.usd          # USD stage operations
import omni.kit.commands # Kit command execution
import omni.timeline     # Timeline control
import omni.kit.app      # Application lifecycle
```

### USD/Physics Modules

```python
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, UsdLux
# - Usd: Core USD functionality
# - UsdGeom: Geometry primitives (Xform, Mesh, Cube, Sphere, etc.)
# - Gf: Math types (Vec3d, Vec3f, Matrix4d, Quatd)
# - Sdf: Scene description (Sdf.Path, Sdf.Layer)
# - UsdPhysics: Physics schema (RigidBodyAPI, CollisionAPI)
# - PhysxSchema: PhysX-specific schemas
# - UsdLux: Lighting (DomeLight)
```

### Isaac Sim Specific Modules

```python
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulation import Gripper
from isaacsim.robot import Robot
from isaacsim.scene.graph import Graph
```

### ROS2 Bridge Module

```python
from isaacsim.ros2.bridge import Ros2Bridge
# Enables ROS2 communication (publish/subscribe)
```

### Warehouse/CuOpt Modules

```python
from omni.cuopt.visualization.generate_warehouse_building import generate_building_structure
from omni.cuopt.visualization.generate_warehouse_assets import generate_shelves_assets, generate_conveyor_assets
from omni.cuopt.visualization.generate_waypoint_graph import visualize_waypoint_graph
from omni.cuopt.visualization.common import check_build_base_path
from omni.cuopt.service.waypoint_graph_model import WaypointGraphModel, load_waypoint_graph_from_file
```

### Physics Modules

```python
from omni.physx import get_physx_interface
from omni.physx.scripts.utils import set_drive_velocity
```

---

## Running Python Scripts in Isaac Sim

### Command Line Execution

Use the `--exec` flag to run Python scripts:

```bash
# Basic execution
./isaac-sim.sh --headless --exec /path/to/script.py

# With extensions enabled
./isaac-sim.sh --headless \
    --enable isaacsim.ros2.bridge \
    --enable omni.cuopt.examples \
    --enable omni.cuopt.visualization \
    --enable omni.cuopt.service \
    --exec /path/to/script.py
```

### Key Flags

| Flag | Description |
|------|-------------|
| `--headless` | Run without GUI |
| `--exec SCRIPT` | Execute Python script |
| `--enable EXT` | Enable extension by name |
| `--list-exts` | List all available extensions |
| `--/key=value` | Override config setting |

### Script Structure for Headless Mode

```python
#!/usr/bin/env python3
"""Isaac Sim Headless Script Template"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.kit.app

def run_main():
    """Main function - runs after Kit is ready"""
    # Get USD stage
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    # Your scene creation code here
    print("Creating scene...")

def on_kit_ready():
    """Called when Kit initialization is complete"""
    run_main()

def setup_update_subscription():
    """Subscribe to startup events"""
    app = omni.kit.app.get_app()
    
    if app.is_running():
        on_kit_ready()
        return
    
    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda event: on_kit_ready() 
        if event.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE) 
        else None
    )

if __name__ == "__main__":
    setup_update_subscription()
```

---

## Core APIs for Scene Creation

### Getting the USD Stage

```python
# Get the current USD stage
usd_context = omni.usd.get_context()
stage = usd_context.get_stage()

# Create a new stage (optional)
new_stage = Usd.Stage.CreateNew("/path/to/new_stage.usd")
```

### Creating Primitives

```python
import omni.kit.commands
from pxr import UsdGeom, Gf

# Create a prim at a path
omni.kit.commands.execute(
    "CreatePrim",
    prim_path="/World/MyObject",
    prim_type="Xform",  # Or "Cube", "Sphere", "Mesh", etc.
    select_new_prim=False
)

# Create a cube with position
cube_path = "/World/MyCube"
omni.kit.commands.execute(
    "CreatePrim",
    prim_path=cube_path,
    prim_type="Cube",
    select_new_prim=False
)

# Set transform (position, rotation, scale)
prim = stage.GetPrimAtPath(cube_path)
if prim:
    xform = UsdGeom.Xformable(prim)
    
    # Get existing ops (Cube has defaults)
    existing_ops = xform.GetOrderedXformOps()
    
    # Update position
    for op in existing_ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(10.0, 5.0, 0.0))
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(0.5, 0.5, 0.5))
```

### Setting Colors/Materials

```python
from pxr import UsdGeom, Gf

# Set display color
prim = stage.GetPrimAtPath(cube_path)
UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])  # Red
```

### Adding Physics

```python
from pxr import UsdPhysics

# Add rigid body
prim = stage.GetPrimAtPath(cube_path)
UsdPhysics.RigidBodyAPI.Apply(prim)

# Add collision
UsdPhysics.CollisionAPI.Apply(prim)

# Add mass (optional)
UsdPhysics.MassAPI.Apply(prim)
UsdPhysics.MassAPI(prim).GetMassAttr().Set(1.0)  # 1 kg
```

### Loading USD Assets

```python
# Reference an existing USD file
prim_path = "/World/LoadedAsset"
omni.kit.commands.execute(
    "CreatePrim",
    prim_path=prim_path,
    prim_type="Xform"
)

prim = stage.GetPrimAtPath(prim_path)
reference_path = "/path/to/asset.usd"
prim.GetReferences().AddReference(reference_path)
```

### Setting Camera View

```python
from isaacsim.core.utils.viewports import set_camera_view

set_camera_view(
    eye=[50.0, 0.0, 40.0],      # Camera position
    target=[20.0, 40.0, 0.0],   # Look-at target
    camera_prim_path="/OmniverseKit_Persp"
)
```

### Getting Assets Root Path

```python
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
# Returns: /home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/data/assets

# Use for building asset paths
nvidia_assets = assets_root + "/NVIDIA/Assets/"
```

---

## Sample Scripts

### 1. Basic Scene Creation

```python
#!/usr/bin/env python3
"""Basic scene creation example"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.kit.app
from pxr import UsdGeom, Gf, UsdPhysics

def create_basic_scene():
    """Create a basic scene with ground plane and cube"""
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    # Create ground plane
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/Ground",
        prim_type="Plane",
        select_new_prim=False
    )
    
    # Create cube
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/Cube",
        prim_type="Cube",
        select_new_prim=False
    )
    
    # Position cube
    prim = stage.GetPrimAtPath("/World/Cube")
    xform = UsdGeom.Xformable(prim)
    
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(0, 5, 0))  # 5 units up
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            op.Set(Gf.Vec3f(1, 1, 1))
    
    # Add physics to cube
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    
    print("Basic scene created!")

def on_kit_ready():
    create_basic_scene()

def setup():
    app = omni.kit.app.get_app()
    if app.is_running():
        on_kit_ready()
        return
    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda e: on_kit_ready() 
        if e.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE) 
        else None
    )

if __name__ == "__main__":
    setup()
```

### 2. Loading Warehouse Environment

```python
#!/usr/bin/env python3
"""Load the official NVIDIA warehouse demo"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.kit.app
from pxr import UsdGeom, Gf, UsdPhysics

# Hardcoded path - in production, use extension manager
EXTENSION_DATA_PATH = "/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/extscache/omni.cuopt.examples-1.3.0+107.3.2/omni/cuopt/examples/warehouse_transport_demo/extension_data/"

def get_assets_root_path():
    from isaacsim.storage.native import get_assets_root_path
    return get_assets_root_path()

def load_warehouse():
    """Load warehouse building, shelves, and waypoint graph"""
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    from omni.cuopt.visualization.generate_warehouse_building import generate_building_structure
    from omni.cuopt.visualization.generate_warehouse_assets import generate_shelves_assets
    from omni.cuopt.visualization.generate_waypoint_graph import visualize_waypoint_graph
    from omni.cuopt.visualization.common import check_build_base_path
    from omni.cuopt.service.waypoint_graph_model import load_waypoint_graph_from_file
    
    base_isaac_path = get_assets_root_path()
    base_nvidia_path = base_isaac_path + "/NVIDIA/Assets/"
    
    # Load building
    building_prim_path = "/World/Warehouse/Building"
    check_build_base_path(stage, building_prim_path, final_xform=True)
    generate_building_structure(
        stage,
        building_prim_path,
        EXTENSION_DATA_PATH + "warehouse_building_data.json",
        base_isaac_path,
    )
    
    # Load shelves
    shelves_prim_path = "/World/Warehouse/Assets/Shelves"
    check_build_base_path(stage, shelves_prim_path, final_xform=True)
    generate_shelves_assets(
        stage,
        shelves_prim_path,
        EXTENSION_DATA_PATH + "warehouse_shelves_data.json",
        base_nvidia_path,
    )
    
    # Load waypoint graph
    waypoint_graph_node_path = "/World/Warehouse/Transportation/WaypointGraph/Nodes"
    waypoint_graph_edge_path = "/World/Warehouse/Transportation/WaypointGraph/Edges"
    
    waypoint_graph_model = load_waypoint_graph_from_file(
        stage, EXTENSION_DATA_PATH + "waypoint_graph.json"
    )
    visualize_waypoint_graph(
        stage,
        waypoint_graph_model,
        waypoint_graph_node_path,
        waypoint_graph_edge_path,
    )
    
    print("Warehouse loaded!")

def on_kit_ready():
    load_warehouse()

def setup():
    app = omni.kit.app.get_app()
    if app.is_running():
        on_kit_ready()
        return
    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda e: on_kit_ready() 
        if e.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE) 
        else None
    )

if __name__ == "__main__":
    setup()
```

### 3. Creating AMR Robots with Physics

```python
#!/usr/bin/env python3
"""Spawn AMR robots in the warehouse"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.kit.app
from pxr import UsdGeom, Gf, UsdPhysics

def spawn_robots(stage, num_robots=3):
    """Spawn AMR robots at specified positions"""
    
    robot_positions = [
        {"name": "amr1", "x": 17.98, "y": 4.16, "z": 0.0},  # charging_station
        {"name": "amr2", "x": 17.98, "y": 8.79, "z": 0.0},  # staging_area
        {"name": "amr3", "x": 17.98, "y": 47.7, "z": 0.0}, # asrs_output
    ]
    
    for i, robot in enumerate(robot_positions[:num_robots]):
        robot_path = f"/World/AMR/{robot['name']}"
        cube_path = f"{robot_path}/Body"
        
        # Create robot body (cube as placeholder)
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=cube_path,
            prim_type="Cube",
            select_new_prim=False
        )
        
        prim = stage.GetPrimAtPath(cube_path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            
            # Update transform
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(robot["x"], robot["y"], robot["z"]))
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    op.Set(Gf.Vec3f(0.5, 0.3, 0.2))
            
            # Set color
            colors = [
                (1.0, 0.3, 0.3),  # Red
                (0.3, 1.0, 0.3),  # Green
                (0.3, 0.3, 1.0),  # Blue
            ]
            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*colors[i])])
            
            # Add physics
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            
            print(f"Spawned {robot['name']} at ({robot['x']}, {robot['y']})")

def on_kit_ready():
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    spawn_robots(stage, num_robots=3)

def setup():
    app = omni.kit.app.get_app()
    if app.is_running():
        on_kit_ready()
        return
    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda e: on_kit_ready() 
        if e.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE) 
        else None
    )

if __name__ == "__main__":
    setup()
```

### 4. Complete Warehouse with Robots

This is the complete script from our implementation:

```python
#!/usr/bin/env python3
"""
Isaac Sim Warehouse Loading Script - Headless Compatible
Loads the official NVIDIA warehouse demo with proper initialization.

Usage:
    ./isaac-sim.sh --headless \
        --enable omni.cuopt.examples \
        --enable omni.cuopt.visualization \
        --enable omni.cuopt.service \
        --enable isaacsim.ros2.bridge \
        --exec amr_description/scripts/isaac_warehouse_headless.py
"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.timeline
import omni.kit.app
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics

EXTENSION_DATA_PATH = None

def get_extension_data_path():
    """Get the path to the warehouse demo extension data."""
    global EXTENSION_DATA_PATH
    if EXTENSION_DATA_PATH is not None:
        return EXTENSION_DATA_PATH
    
    # Use known path since extension path isn't resolved in --exec mode
    EXTENSION_DATA_PATH = "/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/extscache/omni.cuopt.examples-1.3.0+107.3.2/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
    return EXTENSION_DATA_PATH

def load_warehouse_environment(stage):
    """Load the warehouse building, shelves, and conveyors."""
    from omni.cuopt.visualization.generate_warehouse_assets import (
        generate_shelves_assets,
        generate_conveyor_assets,
    )
    from omni.cuopt.visualization.generate_warehouse_building import (
        generate_building_structure,
    )
    from omni.cuopt.visualization.common import check_build_base_path
    from isaacsim.storage.native import get_assets_root_path

    print("[Isaac Sim] Loading warehouse environment...")

    ext_data_path = get_extension_data_path()
    building_config = "warehouse_building_data.json"
    shelves_config = "warehouse_shelves_data.json"
    conveyors_config = "warehouse_conveyors_data.json"

    base_isaac_path = get_assets_root_path()
    base_nvidia_path = base_isaac_path + "/NVIDIA/Assets/"
    digital_twin_path = base_nvidia_path + "DigitalTwin/Assets/Warehouse/"

    building_prim_path = "/World/Warehouse/Building"
    check_build_base_path(stage, building_prim_path, final_xform=True)
    generate_building_structure(stage, building_prim_path, ext_data_path + building_config, base_isaac_path)

    shelves_prim_path = "/World/Warehouse/Assets/Shelves"
    check_build_base_path(stage, shelves_prim_path, final_xform=True)
    generate_shelves_assets(stage, shelves_prim_path, ext_data_path + shelves_config, base_nvidia_path)

    conveyor_prim_path = "/World/Warehouse/Assets/Conveyors"
    check_build_base_path(stage, conveyor_prim_path, final_xform=True)
    generate_conveyor_assets(stage, conveyor_prim_path, ext_data_path + conveyors_config, digital_twin_path)

    print("[Isaac Sim] Warehouse environment loaded!")

def load_waypoint_graph(stage):
    """Load the 90-waypoint graph."""
    from omni.cuopt.visualization.generate_waypoint_graph import visualize_waypoint_graph
    from omni.cuopt.service.waypoint_graph_model import load_waypoint_graph_from_file

    print("[Isaac Sim] Loading waypoint graph...")
    ext_data_path = get_extension_data_path()
    waypoint_graph_config = "waypoint_graph.json"

    waypoint_graph_node_path = "/World/Warehouse/Transportation/WaypointGraph/Nodes"
    waypoint_graph_edge_path = "/World/Warehouse/Transportation/WaypointGraph/Edges"

    waypoint_graph_model = load_waypoint_graph_from_file(stage, ext_data_path + waypoint_graph_config)
    visualize_waypoint_graph(stage, waypoint_graph_model, waypoint_graph_node_path, waypoint_graph_edge_path)

    print("[Isaac Sim] Waypoint graph loaded!")

def spawn_amr_robots(stage, num_robots=3):
    """Spawn AMR robots in the warehouse."""
    print(f"[Isaac Sim] Spawning {num_robots} AMR robots...")

    robot_positions = [
        {"name": "amr1", "x": 17.98, "y": 4.16, "z": 0.0},
        {"name": "amr2", "x": 17.98, "y": 8.79, "z": 0.0},
        {"name": "amr3", "x": 17.98, "y": 47.7, "z": 0.0},
    ]

    for i, robot in enumerate(robot_positions[:num_robots]):
        robot_path = f"/World/AMR/{robot['name']}"
        cube_path = f"{robot_path}/Body"

        omni.kit.commands.execute(
            "CreatePrim", prim_path=cube_path, prim_type="Cube", select_new_prim=False
        )

        prim = stage.GetPrimAtPath(cube_path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            
            # Update existing translate/scale ops
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(robot["x"], robot["y"], robot["z"]))
                elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    op.Set(Gf.Vec3f(0.5, 0.3, 0.2))

            colors = [(1.0, 0.3, 0.3), (0.3, 1.0, 0.3), (0.3, 0.3, 1.0)]
            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*colors[i])])

            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)

            print(f"[Isaac Sim] Spawned {robot['name']} at ({robot['x']}, {robot['y']})")

    print(f"[Isaac Sim] {num_robots} AMR robots spawned!")

def run_main():
    """Main function - runs after Isaac Sim is fully initialized."""
    print("=" * 60)
    print("[Isaac Sim] Warehouse ROS2 Loading Script")
    print("=" * 60)

    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    if stage is None:
        print("[Isaac Sim] ERROR: No USD stage available!")
        return

    load_warehouse_environment(stage)
    load_waypoint_graph(stage)
    spawn_amr_robots(stage, num_robots=3)

    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[50.0, 0.0, 40.0], target=[20.0, 40.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

    print("=" * 60)
    print("[Isaac Sim] Warehouse loaded successfully!")
    print("[Isaac Sim] Ready for ROS2 connection")
    print("=" * 60)

def on_kit_ready():
    """Called when Kit is ready."""
    print("[Isaac Sim] Kit ready, loading warehouse...")
    run_main()

def setup_update_subscription():
    """Subscribe to update events to run after initialization."""
    app = omni.kit.app.get_app()
    
    if app.is_running():
        on_kit_ready()
        return

    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda event: on_kit_ready() if event.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE) else None
    )

if __name__ == "__main__":
    setup_update_subscription()
```

---

## Extension Data Paths

### CuOpt Warehouse Demo

```
/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/extscache/omni.cuopt.examples-1.3.0+107.3.2/omni/cuopt/examples/warehouse_transport_demo/extension_data/
```

Contains:
- `waypoint_graph.json` - 90 waypoints for navigation
- `warehouse_building_data.json` - Building configuration
- `warehouse_shelves_data.json` - Shelves placement
- `warehouse_conveyors_data.json` - Conveyor belt positions

### NVIDIA Assets

```
/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/data/assets/NVIDIA/Assets/
```

Contains:
- `Isaac/` - Isaac Sim assets
- `DigitalTwin/Assets/Warehouse/` - Warehouse-specific assets

---

## ROS2 Bridge Integration

### Enabling ROS2 Bridge

```bash
./isaac-sim.sh --headless --enable isaacsim.ros2.bridge --exec script.py
```

### ROS2 Topics Available

Once enabled, these topics are available:

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | nav_msgs/Odometry | Robot odometry |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/scan` | sensor_msgs/LaserScan | Lidar scan |
| `/imu` | sensor_msgs/Imu | IMU data |

### Using ROS2 in Python

```python
# The ROS2 bridge is automatically enabled
# You can publish/subscribe using standard ROS2 APIs from Isaac Sim

# Note: rclpy needs to be sourced in the environment
# Isaac Sim includes an embedded rclpy for ROS2 Humble
```

---

## Troubleshooting

### Module Not Found Errors

If you get `ModuleNotFoundError: No module named 'omni.cuopt'`:

```bash
# Make sure to enable the required extensions
./isaac-sim.sh --headless \
    --enable omni.cuopt.examples \
    --enable omni.cuopt.visualization \
    --enable omni.cuopt.service \
    --exec script.py
```

### Extension Path Issues

If `get_extension_path()` returns empty:

```python
# Use hardcoded path as fallback
EXTENSION_DATA_PATH = "/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/extscache/omni.cuopt.examples-1.3.0+107.3.2/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
```

### Physics Not Working

Ensure physics is properly configured:

```python
from pxr import UsdPhysics

# Apply to prim
UsdPhysics.RigidBodyAPI.Apply(prim)
UsdPhysics.CollisionAPI.Apply(prim)
```

### List Available Extensions

```bash
./isaac-sim.sh --headless --list-exts
```

---

## Quick Reference

### Launch Isaac Sim

```bash
cd /home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64
./isaac-sim.sh --headless --exec /path/to/script.py
```

### Common Imports

```python
import omni.usd
import omni.kit.commands
import omni.kit.app
from pxr import UsdGeom, Gf, UsdPhysics
from isaacsim.core.utils.viewports import set_camera_view
```

### Create Prim

```python
omni.kit.commands.execute(
    "CreatePrim",
    prim_path="/World/MyObject",
    prim_type="Xform",  # or "Cube", "Sphere", "Mesh"
    select_new_prim=False
)
```

### Get Stage

```python
stage = omni.usd.get_context().get_stage()
```

---

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/)
- [Isaac Sim Python API Reference](file:///home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64/docs/py/api/index.html)
- [NVIDIA Omniverse Forums](https://forums.omniverse.nvidia.com/)
- [CuOpt Examples](https://docs.nvidia.com/cuopt/)

---

## Robot Assets Available

### iw_hub Robot (Warehouse AMR)

The iw_hub robot is available in Isaac Sim at:
```
/Isaac/Robots/Idealworks/iw_hub.usd
/Isaac/Robots/Idealworks/iw_hub_sensors.usd
```

These paths are relative to the Isaac Sim assets root. Use with `get_assets_root_path()`:

```python
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
iw_hub_usd = f"{assets_root}/Isaac/Robots/Idealworks/iw_hub.usd"
iw_hub_sensors_usd = f"{assets_root}/Isaac/Robots/Idealworks/iw_hub_sensors.usd"
```

### Loading Robots in Script

```python
from isaacsim.storage.native import get_assets_root_path

# Get assets root path (resolves to online Omniverse content)
assets_root = get_assets_root_path()

# iw_hub robot paths
iw_hub_usd = f"{assets_root}/Isaac/Robots/Idealworks/iw_hub.usd"
iw_hub_sensors_usd = f"{assets_root}/Isaac/Robots/Idealworks/iw_hub_sensors.usd"

robot_path = "/World/AMR/amr1"

# Create parent Xform
omni.kit.commands.execute(
    "CreatePrim",
    prim,
    prim_type="Xform",
_path=robot_path    select_new_prim=False
)

# Add robot reference
prim = stage.GetPrimAtPath(robot_path)
prim.GetReferences().AddReference(iw_hub_usd)

# Add sensors
sensor_prim_path = f"{robot_path}/sensors"
omni.kit.commands.execute(
    "CreatePrim",
    prim_path=sensor_prim_path,
    prim_type="Xform",
    select_new_prim=False
)
sensor_prim = stage.GetPrimAtPath(sensor_prim_path)
sensor_prim.GetReferences().AddReference(iw_hub_sensors_usd)
```
