@echo off
REM Launch Isaac Sim with warehouse environment and ROS2 bridge (Windows)
REM Usage: Run this script from Isaac Sim installation directory

set SCRIPT_DIR=C:\Users\xelf\warehouse-automation-001
set ISAAC_SIM_DIR=C:\isaac-sim

echo ==============================================
echo Launching Isaac Sim Warehouse Environment
echo ==============================================

cd "%ISAAC_SIM_DIR%"

REM Run Isaac Sim in headless mode with:
REM - ROS2 bridge enabled (for communication with ROS2)
REM - Custom warehouse loading script (creates full warehouse with floor, walls, shelves, conveyors, waypoints, robots)

.\isaac-sim.bat ^
    --headless ^
    --enable isaacsim.ros2.bridge ^
    --exec "%SCRIPT_DIR%\amr_description\scripts\isaac_warehouse_headless.py"

echo Isaac Sim stopped
pause
