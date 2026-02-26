#!/usr/bin/env python3
"""
Isaac Sim Warehouse Scene with AMR Robots
Creates warehouse environment and spawns AMR robots from URDF
"""

import argparse
import omni.usd
from pxr import Gf, Sdf, UsdGeom
import omni.kit.commands
import omni.timeline

from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.transformations import quat_from_euler_angles

from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path


def create_ground(stage, path="/World/ground"):
    """Create ground plane"""
    create_prim(
        prim_path=path,
        prim_type="Xform",
    )
    ground_prim = stage.GetPrimAtPath(path)
    
    # Add collision and visual
    UsdGeom.Mesh.Apply(ground_prim)
    ground_geom = UsdGeom.Cube(ground_prim)
    ground_geom.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    ground_geom.AddScaleOp().Set(Gf.Vec3f(50.0, 50.0, 0.01))
    
    # Create collision
    prim = stage.GetPrimAtPath(path)
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, -0.05))
    prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(50.0, 50.0, 0.01))


def create_box(stage, path, size, position, color=(0.5, 0.5, 0.5)):
    """Create a box in the scene"""
    create_prim(prim_path=path, prim_type="Xform")
    prim = stage.GetPrimAtPath(path)
    
    geom = UsdGeom.Cube(prim)
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    geom.AddScaleOp().Set(Gf.Vec3f(size[0]/2, size[1]/2, size[2]/2))
    
    # Set color
    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


def create_warehouse_scene(stage):
    """Create warehouse environment with palletizer, AS/RS, staging"""
    
    # Ground plane
    create_ground(stage, "/World/ground")
    
    # Palletizer (orange box) at position -8, 3
    create_box(
        stage, 
        "/World/palletizer",
        size=(2, 2, 1),
        position=(-8, 3, 0.5),
        color=(0.9, 0.4, 0.1)
    )
    
    # AS/RS (blue box) at position 8, 3
    create_box(
        stage,
        "/World/asrs",
        size=(4, 1, 2),
        position=(8, 3, 1),
        color=(0.2, 0.4, 0.9)
    )
    
    # Staging area (green box) at position 0, -5
    create_box(
        stage,
        "/World/staging",
        size=(4, 4, 0.5),
        position=(0, -5, 0.25),
        color=(0.2, 0.9, 0.2)
    )


def spawn_amr_from_urdf(urdf_path, prim_path, position, orientation=(0, 0, 0, 1)):
    """Spawn AMR robot from URDF using Isaac Sim's URDF importer"""
    from isaacsim.assets import check_file_existence
    
    if not check_file_existence(urdf_path):
        print(f"Warning: URDF file not found: {urdf_path}")
        return
    
    # Import URDF using omni.kit.commands
    from omni.kit.commands import execute
    
    execute(
        "CreateURDF",
        usd_format="usd",
        urdf_file_path=urdf_path,
        destination_path=prim_path,
        import_option="visual",
        target_frame_position=(0, 0, 0),
        robot_base_position=(position[0], position[1], position[2]),
        robot_base_orientation=orientation,
    )


def spawn_amrs(urdf_path, num_robots=3):
    """Spawn multiple AMR robots in the warehouse"""
    positions = [
        (-5, 0, 0),
        (-5, 2, 0),
        (-5, -2, 0),
    ]
    
    for i in range(min(num_robots, len(positions))):
        prim_path = f"/World/amr{i+1}"
        pos = positions[i]
        
        # Calculate quaternion from euler (0, 0, 0)
        quat = quat_from_euler_angles(0, 0, 0)
        
        spawn_amr_from_urdf(urdf_path, prim_path, pos, quat)
        print(f"Spawned AMR {i+1} at position {pos}")


def setup_physics(stage):
    """Setup physics scene"""
    from pxr import PhysxSchema
    
    # Create physics scene
    scene_path = "/World/PhysicsScene"
    create_prim(prim_path=scene_path, prim_type="PhysicsScene")
    
    # Get physics scene
    scene = stage.GetPrimAtPath(scene_path)
    if scene:
        PhysxSchema.PhysicsScene.Apply(scene)


def main():
    """Main function to setup Isaac Sim warehouse scene"""
    parser = argparse.ArgumentParser(description="Isaac Sim Warehouse Scene")
    parser.add_argument("--urdf", type=str, 
                        default="/home/xelf/warehouse-automation/install/amr_description/share/amr_description/urdf/amr_robot.urdf",
                        help="Path to AMR URDF file")
    parser.add_argument("--num-robots", type=int, default=3,
                        help="Number of AMR robots to spawn")
    args = parser.parse_args()
    
    # Get current stage
    stage = get_current_stage()
    
    # Setup physics
    setup_physics(stage)
    
    # Create warehouse scene
    print("Creating warehouse scene...")
    create_warehouse_scene(stage)
    
    # Spawn AMR robots
    print(f"Spawning {args.num_robots} AMR robots...")
    spawn_amrs(args.urdf, args.num_robots)
    
    # Set camera view
    omni.timeline.get_timeline_interface().set_end_time(1000000)
    
    print("Warehouse scene setup complete!")


if __name__ == "__main__":
    main()
