#!/usr/bin/env python3
"""
Isaac Sim Warehouse Loading Script - Headless Compatible
Loads the official NVIDIA warehouse demo with proper initialization.

Usage:
    ./isaac-sim.sh --headless --exec amr_description/scripts/isaac_warehouse_headless.py

Or with Kit directly:
    ./kit/kit apps/isaacsim.exp.full.kit --headless --exec /path/to/script.py
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

    print(f"[DEBUG] Extension data path: {EXTENSION_DATA_PATH}")

    return EXTENSION_DATA_PATH

    ext_manager = omni.kit.app.get_app().get_extension_manager()

    # Get path to omni.cuopt.examples extension
    ext_path = ext_manager.get_extension_path("omni.cuopt.examples")
    # The extension path already includes the full path to the extension
    # extension_path/omni/cuopt/examples/warehouse_transport_demo/extension_data/
    EXTENSION_DATA_PATH = (
        f"{ext_path}/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
    )

    print(f"[DEBUG] Extension path: {ext_path}")
    print(f"[DEBUG] Extension data path: {EXTENSION_DATA_PATH}")

    return EXTENSION_DATA_PATH

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_path = ext_manager.get_extension_path("omni.cuopt.examples")
    EXTENSION_DATA_PATH = (
        f"{ext_path}/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
    )
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
    generate_building_structure(
        stage, building_prim_path, ext_data_path + building_config, base_isaac_path
    )

    shelves_prim_path = "/World/Warehouse/Assets/Shelves"
    check_build_base_path(stage, shelves_prim_path, final_xform=True)
    generate_shelves_assets(
        stage, shelves_prim_path, ext_data_path + shelves_config, base_nvidia_path
    )

    conveyor_prim_path = "/World/Warehouse/Assets/Conveyors"
    check_build_base_path(stage, conveyor_prim_path, final_xform=True)
    generate_conveyor_assets(
        stage, conveyor_prim_path, ext_data_path + conveyors_config, digital_twin_path
    )

    print("[Isaac Sim] Warehouse environment loaded!")


def load_waypoint_graph(stage):
    """Load the 90-waypoint graph."""
    from omni.cuopt.visualization.generate_waypoint_graph import (
        visualize_waypoint_graph,
    )
    from omni.cuopt.service.waypoint_graph_model import load_waypoint_graph_from_file

    print("[Isaac Sim] Loading waypoint graph...")
    ext_data_path = get_extension_data_path()
    waypoint_graph_config = "waypoint_graph.json"

    waypoint_graph_node_path = "/World/Warehouse/Transportation/WaypointGraph/Nodes"
    waypoint_graph_edge_path = "/World/Warehouse/Transportation/WaypointGraph/Edges"

    waypoint_graph_model = load_waypoint_graph_from_file(
        stage, ext_data_path + waypoint_graph_config
    )
    visualize_waypoint_graph(
        stage, waypoint_graph_model, waypoint_graph_node_path, waypoint_graph_edge_path
    )

    print("[Isaac Sim] Waypoint graph loaded!")


def spawn_amr_robots(stage, num_robots=3):
    """Spawn AMR robots (iw_hub) with sensors in the warehouse."""
    from pxr import PhysxSchema, Sdf

    print(f"[Isaac Sim] Spawning {num_robots} AMR robots with sensors...")

    # Use local iw_hub assets (in other_assets folder)
    iw_hub_usd = "/home/ubuntu/isaac-sim-allfiles/other_assets/iw_hub.usd"
    iw_hub_sensors_usd = (
        "/home/ubuntu/isaac-sim-allfiles/other_assets/iw_hub_sensors.usd"
    )

    print(f"[Isaac Sim] iw_hub path: {iw_hub_usd}")
    print(f"[Isaac Sim] iw_hub sensors path: {iw_hub_sensors_usd}")

    robot_positions = [
        {"name": "amr1", "x": 17.98, "y": 4.16, "z": 0.0},
        {"name": "amr2", "x": 17.98, "y": 8.79, "z": 0.0},
        {"name": "amr3", "x": 17.98, "y": 47.7, "z": 0.0},
    ]

    for i, robot in enumerate(robot_positions[:num_robots]):
        robot_path = f"/World/AMR/{robot['name']}"

        print(f"[Isaac Sim] Loading {robot['name']} from {iw_hub_usd}")

        # Create a parent Xform for the robot
        omni.kit.commands.execute(
            "CreatePrim", prim_path=robot_path, prim_type="Xform", select_new_prim=False
        )

        # Add reference to iw_hub USD
        try:
            prim = stage.GetPrimAtPath(robot_path)
            if prim:
                # Add reference to iw_hub robot
                prim.GetReferences().AddReference(iw_hub_usd)

                # Also add sensors if available
                sensor_prim_path = f"{robot_path}/sensors"
                omni.kit.commands.execute(
                    "CreatePrim",
                    prim_path=sensor_prim_path,
                    prim_type="Xform",
                    select_new_prim=False,
                )
                sensor_prim = stage.GetPrimAtPath(sensor_prim_path)
                if sensor_prim:
                    sensor_prim.GetReferences().AddReference(iw_hub_sensors_usd)

                # Set position using Xformable
                # Try to find the chassis link and set position
                chassis_path = f"{robot_path}/chassis_link"
                chassis_prim = stage.GetPrimAtPath(chassis_path)
                if not chassis_path:
                    # Try alternative path
                    chassis_path = f"{robot_path}/base_link"
                    chassis_prim = stage.GetPrimAtPath(chassis_path)

                if chassis_prim:
                    xform = UsdGeom.Xformable(chassis_prim)
                    for op in xform.GetOrderedXformOps():
                        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                            current_pos = op.Get()
                            op.Set(
                                Gf.Vec3d(robot["x"], robot["z"], current_pos[1])
                            )  # Swap Y and Z for Isaac Sim

                print(
                    f"[Isaac Sim] Loaded {robot['name']} at ({robot['x']}, {robot['y']})"
                )
            else:
                print(f"[Isaac Sim] Warning: Could not create prim at {robot_path}")
                print(f"[Isaac Sim] Warning: Could not create prim at {robot_path}")
        except Exception as e:
            print(f"[Isaac Sim] Error loading {robot['name']}: {e}")
            # Fallback to simple cube
            cube_path = f"{robot_path}/Body"
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path=cube_path,
                prim_type="Cube",
                select_new_prim=False,
            )

            prim = stage.GetPrimAtPath(cube_path)
            if prim:
                xform = UsdGeom.Xformable(prim)
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(robot["x"], robot["y"], robot["z"]))
                    elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                        op.Set(Gf.Vec3f(0.5, 0.3, 0.2))

                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.RigidBodyAPI.Apply(prim)
                print(
                    f"[Isaac Sim] Fallback: Spawned {robot['name']} cube at ({robot['x']}, {robot['y']})"
                )

    print(f"[Isaac Sim] {num_robots} AMR robots spawned!")


def add_sensors_to_robot(stage, robot_name, robot_path):
    """Add sensors (lidar, camera, IMU) to a robot."""
    print(f"[Isaac Sim] Adding sensors to {robot_name}...")

    # Use omni.kit.commands to create sensor frames
    # Note: Full sensor integration requires isaacsim.sensor extension

    # Add a lidar sensor frame
    lidar_path = f"{robot_path}/sensors/lidar"
    omni.kit.commands.execute(
        "CreatePrim", prim_path=lidar_path, prim_type="Xform", select_new_prim=False
    )

    # Add camera sensor frame
    camera_front_path = f"{robot_path}/sensors/camera_front"
    camera_rear_path = f"{robot_path}/sensors/camera_rear"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=camera_front_path,
        prim_type="Xform",
        select_new_prim=False,
    )
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=camera_rear_path,
        prim_type="Xform",
        select_new_prim=False,
    )

    # Add IMU sensor frame
    imu_path = f"{robot_path}/sensors/imu"
    omni.kit.commands.execute(
        "CreatePrim", prim_path=imu_path, prim_type="Xform", select_new_prim=False
    )

    print(f"[Isaac Sim] Sensors added to {robot_name}")
    print(f"[Isaac Sim]   - Lidar: {lidar_path}")
    print(f"[Isaac Sim]   - Camera Front: {camera_front_path}")
    print(f"[Isaac Sim]   - Camera Rear: {camera_rear_path}")
    print(f"[Isaac Sim]   - IMU: {imu_path}")


def setup_ros2_bridge():
    """Setup ROS2 bridge with differential drive controller for each robot."""
    import omni.graph.core as og
    from pxr import UsdGeom, Gf, Sdf
    import usdrt

    print("[Isaac Sim] Setting up ROS2 bridge with differential drive controllers...")

    robots = [
        {"name": "amr1", "path": "/World/AMR/amr1"},
        {"name": "amr2", "path": "/World/AMR/amr2"},
        {"name": "amr3", "path": "/World/AMR/amr3"},
    ]

    for robot in robots:
        robot_name = robot["name"]
        robot_path = robot["path"]
        graph_path = f"/Graph/ROS2_{robot_name}"

        print(f"[Isaac Sim] Creating ROS2 graph for {robot_name} at {graph_path}")

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        (
                            "publishRawTF",
                            "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                        ),
                        ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        (
                            "diffController",
                            "isaacsim.robot.wheeled_robots.DifferentialController",
                        ),
                        (
                            "artController",
                            "isaacsim.core.nodes.IsaacArticulationController",
                        ),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "publishOdom.inputs:timeStamp",
                        ),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "publishRawTF.inputs:timeStamp",
                        ),
                        (
                            "computeOdom.outputs:angularVelocity",
                            "publishOdom.inputs:angularVelocity",
                        ),
                        (
                            "computeOdom.outputs:linearVelocity",
                            "publishOdom.inputs:linearVelocity",
                        ),
                        (
                            "computeOdom.outputs:orientation",
                            "publishOdom.inputs:orientation",
                        ),
                        ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                        (
                            "computeOdom.outputs:orientation",
                            "publishRawTF.inputs:rotation",
                        ),
                        (
                            "computeOdom.outputs:position",
                            "publishRawTF.inputs:translation",
                        ),
                        ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                        (
                            "subscribeTwist.outputs:execOut",
                            "diffController.inputs:execIn",
                        ),
                        (
                            "subscribeTwist.outputs:linearVelocity",
                            "breakLinVel.inputs:tuple",
                        ),
                        (
                            "breakLinVel.outputs:x",
                            "diffController.inputs:linearVelocity",
                        ),
                        (
                            "subscribeTwist.outputs:angularVelocity",
                            "breakAngVel.inputs:tuple",
                        ),
                        (
                            "breakAngVel.outputs:z",
                            "diffController.inputs:angularVelocity",
                        ),
                        (
                            "diffController.outputs:velocityCommand",
                            "artController.inputs:velocityCommand",
                        ),
                    ],
                    keys.SET_VALUES: [
                        # Differential drive parameters (adjust for iw_hub)
                        ("diffController.inputs:wheelRadius", 0.1),
                        ("diffController.inputs:wheelDistance", 0.4),
                        # Joint names - need to check actual joint names from iw_hub
                        (
                            "artController.inputs:jointNames",
                            ["left_wheel_joint", "right_wheel_joint"],
                        ),
                        # Robot chassis path
                        (
                            "computeOdom.inputs:chassisPrim",
                            [usdrt.Sdf.Path(robot_path)],
                        ),
                        (
                            "artController.inputs:targetPrim",
                            [usdrt.Sdf.Path(robot_path)],
                        ),
                        # ROS2 topic names - use robot name as namespace
                        ("subscribeTwist.inputs:topicName", f"/{robot_name}/cmd_vel"),
                        ("publishOdom.inputs:topicName", f"/{robot_name}/odom"),
                        ("publishRawTF.inputs:topicName", f"/{robot_name}/tf"),
                        ("publishRawTF.inputs:childFrameId", robot_name),
                        ("publishRawTF.inputs:parentFrameId", "odom"),
                    ],
                },
            )
            print(f"[Isaac Sim] Created ROS2 graph for {robot_name}")
        except Exception as e:
            print(f"[Isaac Sim] Error creating graph for {robot_name}: {e}")

    print("[Isaac Sim] ROS2 bridge setup complete!")
    print("[Isaac Sim] Available topics:")
    for robot in robots:
        print(f"[Isaac Sim]   /{robot['name']}/cmd_vel (subscribe)")
        print(f"[Isaac Sim]   /{robot['name']}/odom (publish)")
        print(f"[Isaac Sim]   /{robot['name']}/tf (publish)")


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
    num_robots = 3
    spawn_amr_robots(stage, num_robots=num_robots)

    # Add sensors to robots
    for i in range(1, num_robots + 1):
        add_sensors_to_robot(stage, f"amr{i}", f"/World/AMR/amr{i}")

    setup_ros2_bridge()

    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(
        eye=[50.0, 0.0, 40.0],
        target=[20.0, 40.0, 0.0],
        camera_prim_path="/OmniverseKit_Persp",
    )

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

    # Check if app is already running
    if app.is_running():
        on_kit_ready()
        return

    # Subscribe to startup complete
    stream = app.get_startup_event_stream()
    subscription = stream.create_subscription_to_pop(
        lambda event: on_kit_ready()
        if event.type == int(omni.kit.app.StartupEvent.BUILD_COMPLETE)
        else None
    )


if __name__ == "__main__":
    setup_update_subscription()
