#!/bin/bash
# Start Gazebo with ROS plugins

source /opt/ros/humble/setup.bash

gzserver $(ros2 pkg prefix amr_description)/share/amr_description/worlds/warehouse.world \
    -slibgazebo_ros_init.so \
    -slibgazebo_ros_factory.so \
    -slibgazebo_ros_force_system.so
