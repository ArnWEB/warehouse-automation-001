#!/usr/bin/env python3
"""
Isaac Sim Warehouse Scene - Simple Version
Creates warehouse environment with AMR robots using basic shapes
"""

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import omni.kit.commands
import omni.timeline
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage


def create_ground(stage, path="/World/ground"):
    """Create ground plane"""
    create_prim(prim_path=path, prim_type="Xform")
    
    # Create plane mesh
    plane_path = f"{path}/Plane"
    create_prim(prim_path=plane_path, prim_type="Plane")
    
    # Set as static rigid body
    prim = stage.GetPrimAtPath(plane_path)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(0.5, 0.5, 0.5)])


def create_box(stage, path, size, position, color=(0.5, 0.5, 0.5)):
    """Create a box in the scene"""
    create_prim(prim_path=path, prim_type="Xform")
    prim = stage.GetPrimAtPath(path)
    
    # Create cube geometry
    geom = UsdGeom.Cube(prim)
    geom.AddTranslateOp().Set(Gf.Vec3d(*position))
    geom.AddScaleOp().Set(Gf.Vec3f(size[0]/2, size[1]/2, size[2]/2))
    
    # Set color
    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    
    # Add collision
    UsdPhysics.CollisionAPI.Apply(prim)


def create_amr_robot(stage, path, position):
    """Create AMR robot using basic shapes"""
    create_prim(prim_path=path, prim_type="Xform")
    
    # Base platform
    base_path = f"{path}/base"
    create_prim(prim_path=base_path, prim_type="Cube")
    base_prim = stage.GetPrimAtPath(base_path)
    
    # Scale to robot base size
    geom = UsdGeom.Cube(base_prim)
    geom.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.1))
    geom.AddScaleOp().Set(Gf.Vec3f(0.6, 0.4, 0.1))
    
    # Color
    UsdGeom.Gprim(base_prim).GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.8, 0.2)])
    
    # Add collision
    UsdPhysics.CollisionAPI.Apply(base_prim)
    
    # Add wheels (4 small cylinders)
    wheel_positions = [
        (0.5, 0.3, 0.05),
        (0.5, -0.3, 0.05),
        (-0.5, 0.3, 0.05),
        (-0.5, -0.3, 0.05),
    ]
    
    for i, wheel_pos in enumerate(wheel_positions):
        wheel_path = f"{path}/wheel{i}"
        create_prim(prim_path=wheel_path, prim_type="Cylinder")
        wheel_prim = stage.GetPrimAtPath(wheel_path)
        
        wheel_geom = UsdGeom.Cylinder(wheel_prim)
        wheel_geom.AddTranslateOp().Set(Gf.Vec3d(*wheel_pos))
        wheel_geom.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.05))
        wheel_geom.AddRotateXOp().Set(90)
        
        # Black color for wheels
        UsdGeom.Gprim(wheel_prim).GetDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.1, 0.1)])
        
        # Add collision
        UsdPhysics.CollisionAPI.Apply(wheel_prim)


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


def spawn_amrs(stage, num_robots=3):
    """Spawn multiple AMR robots in the warehouse"""
    positions = [
        (-5, 0, 0),
        (-5, 2, 0),
        (-5, -2, 0),
    ]
    
    for i in range(min(num_robots, len(positions))):
        prim_path = f"/World/amr{i+1}"
        pos = positions[i]
        
        create_amr_robot(stage, prim_path, pos)
        print(f"Spawned AMR {i+1} at position {pos}")


def setup_physics(stage):
    """Setup physics scene"""
    # Create physics scene
    scene_path = "/World/PhysicsScene"
    create_prim(prim_path=scene_path, prim_type="PhysicsScene")
    
    # Configure gravity
    scene = stage.GetPrimAtPath(scene_path)
    if scene:
        PhysxSchema.PhysicsScene.Apply(scene)


def main():
    """Main function to setup Isaac Sim warehouse scene"""
    import argparse
    parser = argparse.ArgumentParser(description="Isaac Sim Warehouse Scene")
    parser.add_argument("--num-robots", type=int, default=3,
                        help="Number of AMR robots to spawn")
    args = parser.parse_args()
    
    # Get current stage
    stage = get_current_stage()
    
    # Setup physics
    print("Setting up physics...")
    setup_physics(stage)
    
    # Create warehouse scene
    print("Creating warehouse scene...")
    create_warehouse_scene(stage)
    
    # Spawn AMR robots
    print(f"Spawning {args.num_robots} AMR robots...")
    spawn_amrs(stage, args.num_robots)
    
    print("Warehouse scene setup complete!")


if __name__ == "__main__":
    main()
