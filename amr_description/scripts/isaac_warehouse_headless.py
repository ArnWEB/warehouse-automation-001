#!/usr/bin/env python3
"""
Isaac Sim Warehouse Loading Script - Headless Compatible
Creates a complete warehouse environment with floor, walls, shelves, and robots.
"""

import omni.ext
import omni.usd
import omni.kit.commands
import omni.timeline
import omni.kit.app
from pxr import UsdGeom, Gf, UsdLux, UsdPhysics, PhysxSchema, Sdf, PhysicsSchemaTools, UsdShade
import numpy as np


def create_warehouse_floor(stage):
    """Create warehouse floor with physics."""
    print("[Warehouse] Creating warehouse floor...")
    
    size = 100.0
    
    # Create physics ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage,
        "/World/Warehouse/Floor",
        "Z",
        size,
        Gf.Vec3f(0, 0, 0),
        Gf.Vec3f(0.3, 0.3, 0.3)
    )
    
    # Create floor plane for visualization
    omni.kit.commands.execute("CreatePrim", prim_path="/World/Warehouse/FloorPlane", prim_type="Plane")
    floor_plane = stage.GetPrimAtPath("/World/Warehouse/FloorPlane")
    if floor_plane:
        xform = UsdGeom.Xformable(floor_plane)
        xform.AddScaleOp().Set(Gf.Vec3f(size/2, size/2, 1))
        
        # Floor material
        mat_path = "/World/Looks/FloorMaterial"
        mat = UsdShade.Material.Define(stage, mat_path)
        pbr = UsdShade.Shader.Define(stage, f"{mat_path}/PBRShader")
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.35, 0.35, 0.35))
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(floor_plane).Bind(mat)
    
    print(f"[Warehouse] Floor created: {size}m x {size}m")


def create_warehouse_walls(stage):
    """Create warehouse walls."""
    print("[Warehouse] Creating warehouse walls...")
    
    wall_height = 8.0
    wall_thickness = 0.2
    warehouse_length = 80.0
    warehouse_width = 60.0
    
    # Wall material
    mat_path = "/World/Looks/WallMaterial"
    mat = UsdShade.Material.Define(stage, mat_path)
    pbr = UsdShade.Shader.Define(stage, f"{mat_path}/PBRShader")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.7, 0.7, 0.75))
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    
    walls = [
        ("WallNorth", warehouse_length, wall_thickness, wall_height, 0, warehouse_width/2, wall_height/2),
        ("WallSouth", warehouse_length, wall_thickness, wall_height, 0, -warehouse_width/2, wall_height/2),
        ("WallEast", wall_thickness, warehouse_width, wall_height, warehouse_length/2, 0, wall_height/2),
        ("WallWest", wall_thickness, warehouse_width, wall_height, -warehouse_length/2, 0, wall_height/2),
    ]
    
    for name, sx, sy, sz, px, py, pz in walls:
        omni.kit.commands.execute("CreatePrim", prim_path=f"/World/Warehouse/Walls/{name}", prim_type="Cube")
        prim = stage.GetPrimAtPath(f"/World/Warehouse/Walls/{name}")
        if prim:
            xform = UsdGeom.Xformable(prim)
            xform.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
            xform.AddScaleOp().Set(Gf.Vec3f(sx/2, sy/2, sz/2))
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
            UsdPhysics.CollisionAPI.Apply(prim)
    
    print(f"[Warehouse] Created {len(walls)} walls")


def create_shelves(stage):
    """Create warehouse shelving units using Isaac Sim World API."""
    print("[Warehouse] Creating warehouse shelves...")
    
    # Use Isaac Sim World API for easier object creation
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    
    world = World(stage_units_in_meters=1.0)
    
    shelf_positions = []
    for i in range(5):
        shelf_positions.append((-30 + i * 12, 15))
    for i in range(5):
        shelf_positions.append((-30 + i * 12, 0))
    for i in range(5):
        shelf_positions.append((-30 + i * 12, -15))
    
    for idx, (x, y) in enumerate(shelf_positions):
        # Create shelf as a group of boxes
        shelf_height = 4.0
        shelf_width = 2.0
        shelf_depth = 1.0
        thickness = 0.05
        
        # Vertical posts (4 corners)
        for dx, dy in [(-shelf_width/2, -shelf_depth/2), (shelf_width/2, -shelf_depth/2),
                       (-shelf_width/2, shelf_depth/2), (shelf_width/2, shelf_depth/2)]:
            post = world.scene.add(DynamicCuboid(
                prim_path=f"/World/Warehouse/Shelves/Shelf_{idx}/Post_{dx}_{dy}",
                position=np.array([x + dx, y + dy, shelf_height/2]),
                size=thickness * 2,
                color=np.array([0.8, 0.5, 0.2])
            ))
        
        # Horizontal shelves
        for level in range(4):
            z = 1.0 + level * 1.2
            shelf_plane = world.scene.add(DynamicCuboid(
                prim_path=f"/World/Warehouse/Shelves/Shelf_{idx}/Level_{level}",
                position=np.array([x, y, z]),
                scale=np.array([shelf_width, shelf_depth, thickness]),
                size=1.0,
                color=np.array([0.8, 0.5, 0.2])
            ))
    
    print(f"[Warehouse] Created {len(shelf_positions)} shelf units")


def create_conveyors(stage):
    """Create conveyor belt systems."""
    print("[Warehouse] Creating conveyors...")
    
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    
    world = World(stage_units_in_meters=1.0)
    
    conveyors = [
        (35, 20),
        (35, -20),
    ]
    
    for idx, (x, y) in enumerate(conveyors):
        # Belt
        world.scene.add(DynamicCuboid(
            prim_path=f"/World/Warehouse/Conveyors/Conveyor_{idx}/Belt",
            position=np.array([x, y, 0.8]),
            scale=np.array([3.0, 0.5, 0.1]),
            size=1.0,
            color=np.array([0.3, 0.3, 0.3])
        ))
        
        # Legs
        for lx in [-1.2, 1.2]:
            for ly in [-0.3, 0.3]:
                world.scene.add(DynamicCuboid(
                    prim_path=f"/World/Warehouse/Conveyors/Conveyor_{idx}/Leg_{lx}_{ly}",
                    position=np.array([x + lx, y + ly, 0.4]),
                    size=0.1,
                    color=np.array([0.4, 0.4, 0.4])
                ))
    
    print(f"[Warehouse] Created {len(conveyors)} conveyors")


def create_waypoint_graph(stage):
    """Create waypoint graph for navigation using Isaac Sim waypoint API."""
    print("[Warehouse] Creating waypoint graph...")
    
    # Use the NVIDIA waypoint graph system if available
    # Otherwise create visual waypoints
    try:
        from omni.cuopt.visualization.generate_waypoint_graph import visualize_waypoint_graph
        from omni.cuopt.service.waypoint_graph_model import load_waypoint_graph_from_file
        from isaacsim.storage.native import get_assets_root_path
        
        ext_path = "C:/isaacsim/extscache/omni.cuopt.examples-1.3.0+107.3.2/omni/cuopt/examples/warehouse_transport_demo/extension_data/"
        
        waypoint_graph_model = load_waypoint_graph_from_file(stage, ext_path + "waypoint_graph.json")
        visualize_waypoint_graph(
            stage, waypoint_graph_model, 
            "/World/Warehouse/Navigation/WaypointGraph/Nodes",
            "/World/Warehouse/Navigation/WaypointGraph/Edges"
        )
        print("[Warehouse] Waypoint graph loaded from CuOpt")
        return
    except Exception as e:
        print(f"[Warehouse] CuOpt waypoints not available: {e}")
    
    # Fallback: create waypoints manually
    print("[Warehouse] Creating manual waypoints...")
    
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicSphere
    
    world = World(stage_units_in_meters=1.0)
    
    waypoint_spacing = 3.0
    warehouse_length = 80.0
    warehouse_width = 60.0
    
    waypoints = []
    waypoint_id = 0
    
    for x in np.arange(-warehouse_length/2 + 5, warehouse_length/2 - 5, waypoint_spacing):
        for y in np.arange(-warehouse_width/2 + 5, warehouse_width/2 - 5, waypoint_spacing):
            # Skip areas where shelves are
            skip = False
            for shelf_y in [0, 15, -15]:
                if abs(y - shelf_y) < 4:
                    skip = True
            if skip:
                continue
            
            # Use Isaac Sim to create sphere waypoint
            from isaacsim.core.api.objects import DynamicSphere
            
            try:
                world = World(stage_units_in_meters=1.0)
                wp = world.scene.add(DynamicSphere(
                    prim_path=f"/World/Warehouse/Navigation/Waypoints/WP_{waypoint_id}",
                    position=np.array([x, y, 0.1]),
                    radius=0.15,
                    color=np.array([0.2, 0.8, 0.2])
                ))
            except:
                # Fallback to USD
                omni.kit.commands.execute("CreatePrim", prim_path=f"/World/Warehouse/Navigation/Waypoints/WP_{waypoint_id}", prim_type="Sphere")
                prim = stage.GetPrimAtPath(f"/World/Warehouse/Navigation/Waypoints/WP_{waypoint_id}")
                if prim:
                    xform = UsdGeom.Xformable(prim)
                    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.1))
                    xform.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.15))
            
            waypoints.append({"id": waypoint_id, "x": x, "y": y})
            waypoint_id += 1
    
    print(f"[Warehouse] Created {len(waypoints)} waypoints")
    return waypoints


def spawn_amr_robots(stage, num_robots=3):
    """Spawn AMR robots in the warehouse."""
    print(f"[Warehouse] Spawning {num_robots} AMR robots...")
    
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    
    world = World(stage_units_in_meters=1.0)
    
    robot_positions = [
        {"name": "amr1", "x": 15.0, "y": 5.0},
        {"name": "amr2", "x": 15.0, "y": 10.0},
        {"name": "amr3", "x": 15.0, "y": -5.0},
    ]
    
    for robot in robot_positions[:num_robots]:
        # Robot body
        body = world.scene.add(DynamicCuboid(
            prim_path=f"/World/AMR/{robot['name']}/Body",
            position=np.array([robot['x'], robot['y'], 0.15]),
            scale=np.array([0.4, 0.3, 0.15]),
            size=1.0,
            color=np.array([0.2, 0.6, 0.9])
        ))
        
        # Add physics
        prim = stage.GetPrimAtPath(f"/World/AMR/{robot['name']}/Body")
        if prim:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)
        
        print(f"[Warehouse] Spawned {robot['name']} at ({robot['x']}, {robot['y']})")
    
    print(f"[Warehouse] {num_robots} AMR robots spawned!")


def setup_lighting(stage):
    """Setup warehouse lighting."""
    print("[Warehouse] Setting up lighting...")
    
    # Dome light
    omni.kit.commands.execute("CreatePrim", prim_path="/World/Lights/DomeLight", prim_type="DomeLight")
    dome = stage.GetPrimAtPath("/World/Lights/DomeLight")
    if dome:
        dome.GetAttribute("inputs:intensity").Set(500)
    
    # Distant light
    omni.kit.commands.execute("CreatePrim", prim_path="/World/Lights/DistantLight", prim_type="DistantLight")
    distant = stage.GetPrimAtPath("/World/Lights/DistantLight")
    if distant:
        distant.GetAttribute("inputs:intensity").Set(1000)
    
    print("[Warehouse] Lighting setup complete")


def setup_physics_scene(stage):
    """Setup physics scene with gravity."""
    print("[Warehouse] Setting up physics...")
    
    if not stage.GetPrimAtPath("/physicsScene"):
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)
    
    print("[Warehouse] Physics setup complete")


def run_main():
    """Main function - runs after Isaac Sim is fully initialized."""
    print("=" * 60)
    print("[Warehouse] Isaac Sim Warehouse Loading Script")
    print("=" * 60)
    
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    if stage is None:
        print("[Warehouse] ERROR: No USD stage available!")
        return
    
    # Create warehouse environment
    setup_physics_scene(stage)
    create_warehouse_floor(stage)
    create_warehouse_walls(stage)
    create_shelves(stage)
    create_conveyors(stage)
    waypoints = create_waypoint_graph(stage)
    spawn_amr_robots(stage, num_robots=3)
    setup_lighting(stage)
    
    # Set camera view
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(
            eye=[-40.0, -30.0, 40.0],
            target=[20.0, 0.0, 0.0],
            camera_prim_path="/OmniverseKit_Persp",
        )
    except:
        print("[Warehouse] Could not set camera view")
    
    print("=" * 60)
    print("[Warehouse] Warehouse loaded successfully!")
    print(f"[Warehouse] Total waypoints: {len(waypoints) if waypoints else 0}")
    print("[Warehouse] Ready for ROS2 connection")
    print("=" * 60)


def on_kit_ready():
    """Called when Kit is ready."""
    print("[Warehouse] Kit ready, loading warehouse...")
    run_main()


def setup_update_subscription():
    """Subscribe to update events to run after initialization."""
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
