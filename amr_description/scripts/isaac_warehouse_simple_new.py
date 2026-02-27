#!/usr/bin/env python3
"""
Simple Isaac Sim Warehouse Loading Script
Creates a basic warehouse environment with AMR robots for ROS2 control.

Usage:
    cd /home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64
    ./isaac-sim.sh --headless --no-window --script /home/ubuntu/warehouse-automation-001/amr_description/scripts/isaac_warehouse_simple.py
"""

import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf, UsdPhysics


def create_ground(stage, path="/World/ground"):
    """Create ground plane"""
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=path,
        prim_type="Xform",
        select_new_prim=False,
    )

    cube_path = f"{path}/Floor"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=cube_path,
        prim_type="Cube",
        select_new_prim=False,
    )

    prim = stage.GetPrimAtPath(cube_path)
    if prim:
        xform = UsdGeom.Xformable(prim)
        xform.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.01))
        xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.01))
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.3)])
        UsdPhysics.CollisionAPI.Apply(prim)


def create_box(stage, path, size, position, color):
    """Create a box in the scene"""
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=path,
        prim_type="Xform",
        select_new_prim=False,
    )

    cube_path = f"{path}/Box"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=cube_path,
        prim_type="Cube",
        select_new_prim=False,
    )

    prim = stage.GetPrimAtPath(cube_path)
    if prim:
        xform = UsdGeom.Xformable(prim)
        xform.AddScaleOp().Set(Gf.Vec3f(size[0] / 2, size[1] / 2, size[2] / 2))
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(prim)


def create_amr_robot(stage, name, position, color):
    """Create an AMR robot"""
    robot_path = f"/World/AMR/{name}"

    # Create robot base
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=robot_path,
        prim_type="Xform",
        select_new_prim=False,
    )

    # Create body
    body_path = f"{robot_path}/Body"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=body_path,
        prim_type="Cube",
        select_new_prim=False,
    )

    prim = stage.GetPrimAtPath(body_path)
    if prim:
        xform = UsdGeom.Xformable(prim)
        xform.AddScaleOp().Set(Gf.Vec3f(0.5, 0.3, 0.2))
        xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        # Add physics
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)

        # Add mass
        massAPI = UsdPhysics.MassAPI.Apply(prim)
        massAPI.CreateMassAttr(10.0)

    print(f"Created robot {name} at position {position}")


def main():
    print("=" * 60)
    print("[Isaac Sim] Creating Warehouse Environment")
    print("=" * 60)

    # Get USD stage
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()

    if stage is None:
        print("ERROR: No USD stage available!")
        return

    # Create ground
    print("Creating ground plane...")
    create_ground(stage)

    # Create warehouse elements (using new coordinates from waypoint_graph)
    print("Creating warehouse elements...")

    # Inbound dock (waypoint 0: 17.98, 61.49)
    create_box(
        stage, "/World/inbound_dock", (4, 2, 1), (17.98, 61.49, 0.5), (0.9, 0.6, 0.2)
    )

    # Palletizer (waypoint 2: 20.26, 58.46)
    create_box(
        stage, "/World/palletizer", (3, 3, 1.5), (20.26, 58.46, 0.75), (0.9, 0.4, 0.1)
    )

    # AS/RS Storage (waypoint 5: 12.25, 58.46)
    create_box(
        stage, "/World/asrs_storage", (6, 2, 2), (12.25, 58.46, 1.0), (0.2, 0.4, 0.9)
    )

    # AS/RS Input (waypoint 10: 12.25, 53.34)
    create_box(
        stage, "/World/asrs_input", (3, 2, 1), (12.25, 53.34, 0.5), (0.3, 0.5, 0.9)
    )

    # AS/RS Output (waypoint 15: 17.98, 47.7)
    create_box(
        stage, "/World/asrs_output", (3, 2, 1), (17.98, 47.7, 0.5), (0.4, 0.6, 0.9)
    )

    # Staging area (waypoint 75: 17.98, 8.79)
    create_box(
        stage, "/World/staging", (4, 4, 0.1), (17.98, 8.79, 0.05), (0.2, 0.8, 0.2)
    )

    # Outbound dock (waypoint 77: 30.2, 8.79)
    create_box(
        stage, "/World/outbound_dock", (4, 2, 1), (30.2, 8.79, 0.5), (0.8, 0.8, 0.2)
    )

    # Charging station (waypoint 80: 17.98, 4.16)
    create_box(
        stage, "/World/charging", (2, 2, 0.5), (17.98, 4.16, 0.25), (0.9, 0.9, 0.1)
    )

    # Create AMR robots
    print("Creating AMR robots...")

    # Robot 1 - at charging station
    create_amr_robot(stage, "amr1", (17.98, 4.16, 0.2), (1.0, 0.3, 0.3))

    # Robot 2 - at staging
    create_amr_robot(stage, "amr2", (17.98, 8.79, 0.2), (0.3, 1.0, 0.3))

    # Robot 3 - at asrs output
    create_amr_robot(stage, "amr3", (17.98, 47.7, 0.2), (0.3, 0.3, 1.0))

    print("=" * 60)
    print("[Isaac Sim] Warehouse Created Successfully!")
    print("=" * 60)
    print("Robot positions:")
    print("  amr1: (17.98, 4.16)   - charging_station")
    print("  amr2: (17.98, 8.79)   - staging_area")
    print("  amr3: (17.98, 47.7)   - asrs_output")
    print("=" * 60)
    print("\nROS2 Topics will be available when simulation starts:")
    print("  /amr1/cmd_vel, /amr1/odom")
    print("  /amr2/cmd_vel, /amr2/odom")
    print("  /amr3/cmd_vel, /amr3/odom")
    print("=" * 60)


if __name__ == "__main__":
    main()
