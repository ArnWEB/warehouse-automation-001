#!/usr/bin/env python3
"""
Isaac Sim Warehouse Loading Script with ROS2 Bridge
Loads the official NVIDIA warehouse demo and enables ROS2 communication.

Usage:
    ./isaac-sim.sh --headless --script amr_description/scripts/isaac_warehouse_ros2.py

This script:
1. Loads the official warehouse environment (building, shelves, conveyors)
2. Loads the 90-waypoint graph from waypoint_graph.json
3. Enables ROS2 bridge for robot control
4. Spawns AMR robots ready for ROS2 cmd_vel control
"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.timeline
from pxr import UsdGeom, Gf

# Import the official warehouse demo modules
from omni.cuopt.examples.warehouse_transport_demo.extension import (
    cuOptMicroserviceExtension,
)
from omni.cuopt.visualization.generate_waypoint_graph import visualize_waypoint_graph
from omni.cuopt.visualization.generate_warehouse_assets import (
    generate_shelves_assets,
    generate_conveyor_assets,
)
from omni.cuopt.visualization.generate_warehouse_building import (
    generate_building_structure,
)
from omni.cuopt.service.waypoint_graph_model import (
    WaypointGraphModel,
    load_waypoint_graph_from_file,
)
from omni.cuopt.visualization.common import check_build_base_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path


# Get extension data path
EXTENSION_DATA_PATH = None


def get_extension_data_path():
    """Get the path to the warehouse demo extension data."""
    global EXTENSION_DATA_PATH
    if EXTENSION_DATA_PATH is None:
        # Try to find the extension path
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()

        # Look for omni.cuopt.examples extension
        ext_path = ext_manager.get_extension_path("omni.cuopt.examples")
        EXTENSION_DATA_PATH = (
            f"{ext_path}/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
        )

    return EXTENSION_DATA_PATH


def load_warehouse_environment(stage):
    """Load the warehouse building, shelves, and conveyors."""
    print("[Isaac Sim] Loading warehouse environment...")

    ext_data_path = get_extension_data_path()

    building_config = "warehouse_building_data.json"
    shelves_config = "warehouse_shelves_data.json"
    conveyors_config = "warehouse_conveyors_data.json"

    base_isaac_path = get_assets_root_path()
    base_nvidia_path = base_isaac_path + "/NVIDIA/Assets/"
    digital_twin_path = base_nvidia_path + "DigitalTwin/Assets/Warehouse/"

    # Build warehouse structure
    building_prim_path = "/World/Warehouse/Building"
    check_build_base_path(stage, building_prim_path, final_xform=True)

    generate_building_structure(
        stage,
        building_prim_path,
        ext_data_path + building_config,
        base_isaac_path,
    )

    # Add shelves
    shelves_prim_path = "/World/Warehouse/Assets/Shelves"
    check_build_base_path(stage, shelves_prim_path, final_xform=True)

    generate_shelves_assets(
        stage,
        shelves_prim_path,
        ext_data_path + shelves_config,
        base_nvidia_path,
    )

    # Add conveyors
    conveyor_prim_path = "/World/Warehouse/Assets/Conveyors"
    check_build_base_path(stage, conveyor_prim_path, final_xform=True)

    generate_conveyor_assets(
        stage,
        conveyor_prim_path,
        ext_data_path + conveyors_config,
        digital_twin_path,
    )

    # Add lighting
    sky_light_path = "/World/ExteriorHDR"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=sky_light_path,
        prim_type="DomeLight",
        select_new_prim=False,
        attributes={
            UsdLux.Tokens.inputsIntensity: 1000,
            UsdLux.Tokens.inputsSpecular: 1,
            UsdGeom.Tokens.visibility: "inherited",
        },
        create_default_xform=True,
    )

    print("[Isaac Sim] Warehouse environment loaded!")


def load_waypoint_graph(stage):
    """Load the 90-waypoint graph."""
    print("[Isaac Sim] Loading waypoint graph...")

    ext_data_path = get_extension_data_path()
    waypoint_graph_config = "waypoint_graph.json"

    waypoint_graph_node_path = "/World/Warehouse/Transportation/WaypointGraph/Nodes"
    waypoint_graph_edge_path = "/World/Warehouse/Transportation/WaypointGraph/Edges"

    waypoint_graph_model = load_waypoint_graph_from_file(
        stage, ext_data_path + waypoint_graph_config
    )

    visualize_waypoint_graph(
        stage,
        waypoint_graph_model,
        waypoint_graph_node_path,
        waypoint_graph_edge_path,
    )

    print(
        f"[Isaac Sim] Waypoint graph loaded! {len(waypoint_graph_model.get_nodes())} waypoints"
    )


def spawn_amr_robots(stage, num_robots=3):
    """Spawn AMR robots in the warehouse."""
    print(f"[Isaac Sim] Spawning {num_robots} AMR robots...")

    # Robot starting positions (matching our ROS2 config)
    # amr1: charging_station (waypoint 80: 17.98, 4.16)
    # amr2: staging_area (waypoint 75: 17.98, 8.79)
    # amr3: asrs_output (waypoint 15: 17.98, 47.7)

    robot_positions = [
        {"name": "amr1", "x": 17.98, "y": 4.16, "z": 0.0},  # charging_station
        {"name": "amr2", "x": 17.98, "y": 8.79, "z": 0.0},  # staging_area
        {"name": "amr3", "x": 17.98, "y": 47.7, "z": 0.0},  # asrs_output
    ]

    for i, robot in enumerate(robot_positions[:num_robots]):
        robot_path = f"/World/AMR/{robot['name']}"

        # Create robot prim
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=robot_path,
            prim_type="Xform",
            select_new_prim=False,
        )

        # Add a simple cube as robot representation
        cube_path = f"{robot_path}/Body"
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=cube_path,
            prim_type="Cube",
            select_new_prim=False,
        )

        # Set position
        prim = stage.GetPrimAtPath(cube_path)
        if prim:
            xform = UsdGeom.Xformable(prim)
            # Scale down to robot size
            xform.AddScaleOp().Set(Gf.Vec3f(0.5, 0.3, 0.2))
            # Move to position
            xform.AddTranslateOp().Set(Gf.Vec3d(robot["x"], robot["y"], robot["z"]))

            # Set color (different for each robot)
            colors = [
                (1.0, 0.3, 0.3),  # Red
                (0.3, 1.0, 0.3),  # Green
                (0.3, 0.3, 1.0),  # Blue
            ]
            UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*colors[i])])

            # Add collision
            UsdPhysics.CollisionAPI.Apply(prim)

        print(f"[Isaac Sim] Spawned {robot['name']} at ({robot['x']}, {robot['y']})")

    print(f"[Isaac Sim] {num_robots} AMR robots spawned!")


def enable_ros2_bridge():
    """Enable the ROS2 bridge extension."""
    print("[Isaac Sim] Enabling ROS2 bridge...")

    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()

    # Try to enable ROS2 bridge
    try:
        ext_manager.set_extension_enabled("isaacsim.ros2.bridge")
        print("[Isaac Sim] ROS2 bridge enabled!")
    except Exception as e:
        print(f"[Isaac Sim] Warning: Could not enable ROS2 bridge: {e}")
        print("[Isaac Sim] You can enable it manually in the Extensions menu")


def setup_robot_physics(stage, num_robots=3):
    """Add physics to robots so they can move."""
    print("[Isaac Sim] Setting up robot physics...")

    from pxr import UsdPhysics, PhysxSchema

    for i in range(1, num_robots + 1):
        robot_name = f"amr{i}"
        robot_path = f"/World/AMR/{robot_name}/Body"

        prim = stage.GetPrimAtPath(robot_path)
        if prim:
            # Add rigid body API
            UsdPhysics.RigidBodyAPI.Apply(prim)

            # Add collision
            UsdPhysics.CollisionAPI.Apply(prim)

            print(f"[Isaac Sim] Physics enabled for {robot_name}")


def main():
    """Main function to load warehouse and setup ROS2."""
    print("=" * 60)
    print("[Isaac Sim] Warehouse ROS2 Loading Script")
    print("=" * 60)

    # Get USD stage
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    if stage is None:
        print("[Isaac Sim] ERROR: No USD stage available!")
        return

    # Load warehouse environment
    load_warehouse_environment(stage)

    # Load waypoint graph
    load_waypoint_graph(stage)

    # Spawn AMR robots
    spawn_amr_robots(stage, num_robots=3)

    # Setup physics
    setup_robot_physics(stage, num_robots=3)

    # Enable ROS2 bridge
    enable_ros2_bridge()

    # Set camera view
    set_camera_view(
        eye=[50.0, 0.0, 40.0],
        target=[20.0, 40.0, 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )

    print("=" * 60)
    print("[Isaac Sim] Warehouse loaded successfully!")
    print("[Isaac Sim] Ready for ROS2 connection")
    print("=" * 60)
    print("\nTo connect with ROS2:")
    print("  1. Source ROS2: source /opt/ros/humble/setup.bash")
    print("  2. Run task executors: ros2 run orchestrator task_executor")
    print("  3. Monitor: ros2 topic echo /amr1/odom")
    print("=" * 60)


if __name__ == "__main__":
    main()
