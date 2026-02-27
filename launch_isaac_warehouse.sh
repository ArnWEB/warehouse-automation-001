#!/bin/bash
# Launch Isaac Sim with warehouse environment and ROS2 bridge
# Usage: ./launch_isaac_warehouse.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="/home/ubuntu/isaac-sim-allfiles/isaac-sim-standalone-5.1.0-linux-x86_64"

echo "=============================================="
echo "Launching Isaac Sim Warehouse Environment"
echo "=============================================="

cd "$ISAAC_SIM_DIR"

# Run Isaac Sim in headless mode with:
# - ROS2 bridge enabled (for communication with ROS2)
# - CuOpt examples enabled (for warehouse demo)
# - CuOpt visualization enabled (for waypoint graph)
# - CuOpt service enabled
# - Isaac Sim sensors enabled (for lidar, camera, IMU)
# - Custom warehouse loading script
./isaac-sim.sh \
    --headless \
    --enable isaacsim.ros2.bridge \
    --enable omni.cuopt.examples \
    --enable omni.cuopt.visualization \
    --enable omni.cuopt.service \
    --enable isaacsim.sensors.physx \
    --enable omni.isaac.sensor \
    --exec "$SCRIPT_DIR/amr_description/scripts/isaac_warehouse_headless.py"

echo "Isaac Sim stopped"
