#!/usr/bin/env python3
"""
Isaac Lab Warehouse Scene with AMR Robots
Creates warehouse environment and spawns AMR robots using IsaacLab
"""

import argparse
# import omni.isaac.core.utils.prims as prim_utils
from isaaclab.app import AppLauncher

# Launch Isaac Lab
parser = argparse.ArgumentParser(description="Isaac Lab Warehouse Scene")
parser.add_argument("--num-robots", type=int, default=3, help="Number of AMR robots")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Now import IsaacLab modules
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


def design_scene(num_robots=3):
    """Designs the warehouse scene"""
    
    # Ground plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    sim_utils.spawn_ground_plane("/World/ground", cfg_ground)
    
    # Dome light
    cfg_light = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    sim_utils.spawn_light("/World/Light", cfg_light)
    
    # Palletizer (orange box)
    palletizer_cfg = sim_utils.BoxCfg(
        size=(2.0, 2.0, 1.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    )
    sim_utils.spawn_box(
        "/World/palletizer", 
        palletizer_cfg, 
        translation=(-8.0, 3.0, 0.5)
    )
    
    # AS/RS (blue box)
    asrs_cfg = sim_utils.BoxCfg(
        size=(4.0, 1.0, 2.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    )
    sim_utils.spawn_box(
        "/World/asrs", 
        asrs_cfg, 
        translation=(8.0, 3.0, 1.0)
    )
    
    # Staging area (green box)
    staging_cfg = sim_utils.BoxCfg(
        size=(4.0, 4.0, 0.5),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    )
    sim_utils.spawn_box(
        "/World/staging", 
        staging_cfg, 
        translation=(0.0, -5.0, 0.25)
    )
    
    # AMR Robot positions
    positions = [
        (-5.0, 0.0, 0.0),
        (-5.0, 2.0, 0.0),
        (-5.0, -2.0, 0.0),
    ]
    
    # Spawn AMR robots - using Jetbot (small warehouse robot)
    for i in range(num_robots):
        robot_path = f"/World/amr{i+1}"
        pos = positions[i] if i < len(positions) else positions[0]
        
        # Try Jetbot first, fallback to simple box
        try:
            amr_cfg = sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/NVIDIA/Jetbot/jetbot.usd",
            )
            sim_utils.spawn_from_usd(
                robot_path,
                amr_cfg,
                translation=pos,
            )
            print(f"Spawned AMR {i+1} (Jetbot) at {pos}")
        except Exception as e:
            print(f"Failed to spawn Jetbot: {e}")
            # Fallback to simple box as AMR
            amr_cfg = sim_utils.BoxCfg(
                size=(0.4, 0.3, 0.15),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            )
            sim_utils.spawn_box(
                robot_path,
                amr_cfg,
                translation=(pos[0], pos[1], pos[2] + 0.1)
            )
            print(f"Spawned AMR {i+1} (box) at {pos}")
    
    print(f"Warehouse scene created with {num_robots} AMR robots!")


def main():
    """Main function"""
    # Create simulation context
    sim_cfg = SimulationCfg(
        dt=0.005,
        device="cuda:0",
        gravity=(0.0, 0.0, -9.81),
    )
    sim = SimulationContext(sim_cfg)
    
    # Set camera view
    sim.set_camera_view(eye=[15.0, 0.0, 15.0], target=[0.0, 0.0, 0.0])
    
    # Design scene
    print("Creating warehouse scene...")
    design_scene(args.num_robots)
    
    # Reset and play
    sim.reset()
    print("\nPress PLAY in the Isaac Lab UI to start simulation")
    print("Scene is ready!")
    
    # Run simulation
    sim.play()


if __name__ == "__main__":
    main()
