#!/usr/bin/env python3
# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Gazebo Warehouse Waypoint Spawner
Spawns visual waypoint markers matching Isaac Sim coordinates

Usage:
    ros2 run amr_description spawn_waypoints
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import math


WAREHOUSE_LOCATIONS = {
    0: {"name": "waypoint_0", "x": 17.98, "y": 61.49},
    1: {"name": "waypoint_1", "x": 23.34, "y": 58.46},
    2: {"name": "waypoint_2", "x": 20.26, "y": 58.46},
    3: {"name": "waypoint_3", "x": 15.65, "y": 58.46},
    4: {"name": "waypoint_4", "x": 17.98, "y": 58.46},
    5: {"name": "waypoint_5", "x": 12.25, "y": 58.46},
    6: {"name": "waypoint_6", "x": 32.33, "y": 53.34},
    7: {"name": "waypoint_7", "x": 26.77, "y": 53.34},
    8: {"name": "waypoint_8", "x": 23.34, "y": 53.34},
    9: {"name": "waypoint_9", "x": 17.98, "y": 53.34},
    10: {"name": "waypoint_10", "x": 12.25, "y": 53.34},
    11: {"name": "waypoint_11", "x": 8.03, "y": 53.34},
    12: {"name": "waypoint_12", "x": 3.51, "y": 53.34},
    13: {"name": "waypoint_13", "x": 26.77, "y": 49.23},
    14: {"name": "waypoint_14", "x": 23.34, "y": 47.7},
    15: {"name": "waypoint_15", "x": 17.98, "y": 47.7},
    16: {"name": "waypoint_16", "x": 12.25, "y": 47.7},
    17: {"name": "waypoint_17", "x": 7.86, "y": 48.59},
    18: {"name": "waypoint_18", "x": 32.33, "y": 45.83},
    19: {"name": "waypoint_19", "x": 29.45, "y": 45.83},
    20: {"name": "waypoint_20", "x": 3.51, "y": 45.9},
    21: {"name": "waypoint_21", "x": 6.48, "y": 45.9},
    22: {"name": "waypoint_22", "x": 32.33, "y": 44.03},
    23: {"name": "waypoint_23", "x": 29.45, "y": 44.03},
    24: {"name": "waypoint_24", "x": 6.48, "y": 43.96},
    25: {"name": "waypoint_25", "x": 3.51, "y": 43.96},
    26: {"name": "waypoint_26", "x": 32.33, "y": 39.94},
    27: {"name": "waypoint_27", "x": 34.42, "y": 39.94},
    28: {"name": "waypoint_28", "x": 28.22, "y": 39.94},
    29: {"name": "waypoint_29", "x": 28.81, "y": 41.67},
    30: {"name": "waypoint_30", "x": 23.34, "y": 42.65},
    31: {"name": "waypoint_31", "x": 17.98, "y": 42.65},
    32: {"name": "waypoint_32", "x": 12.25, "y": 42.65},
    33: {"name": "waypoint_33", "x": 23.34, "y": 39.94},
    34: {"name": "waypoint_34", "x": 17.98, "y": 39.94},
    35: {"name": "waypoint_35", "x": 12.25, "y": 39.94},
    36: {"name": "waypoint_36", "x": 9.29, "y": 39.94},
    37: {"name": "waypoint_37", "x": 3.51, "y": 39.94},
    38: {"name": "waypoint_38", "x": 1.48, "y": 39.94},
    39: {"name": "waypoint_39", "x": 34.42, "y": 35.29},
    40: {"name": "waypoint_40", "x": 32.33, "y": 35.29},
    41: {"name": "waypoint_41", "x": 30.2, "y": 35.29},
    42: {"name": "waypoint_42", "x": 23.34, "y": 35.29},
    43: {"name": "waypoint_43", "x": 17.98, "y": 35.29},
    44: {"name": "waypoint_44", "x": 12.25, "y": 35.29},
    45: {"name": "waypoint_45", "x": 9.29, "y": 35.29},
    46: {"name": "waypoint_46", "x": 3.51, "y": 35.29},
    47: {"name": "waypoint_47", "x": 1.48, "y": 35.29},
    48: {"name": "waypoint_48", "x": 34.42, "y": 30.55},
    49: {"name": "waypoint_49", "x": 32.33, "y": 30.55},
    50: {"name": "waypoint_50", "x": 28.22, "y": 30.55},
    51: {"name": "waypoint_51", "x": 23.34, "y": 30.55},
    52: {"name": "waypoint_52", "x": 17.98, "y": 30.55},
    53: {"name": "waypoint_53", "x": 12.25, "y": 30.55},
    54: {"name": "waypoint_54", "x": 9.29, "y": 30.55},
    55: {"name": "waypoint_55", "x": 3.51, "y": 30.55},
    56: {"name": "waypoint_56", "x": 32.33, "y": 26.67},
    57: {"name": "waypoint_57", "x": 28.22, "y": 26.67},
    58: {"name": "waypoint_58", "x": 23.34, "y": 26.67},
    59: {"name": "waypoint_59", "x": 17.98, "y": 26.67},
    60: {"name": "waypoint_60", "x": 12.25, "y": 26.67},
    61: {"name": "waypoint_61", "x": 9.29, "y": 26.67},
    62: {"name": "waypoint_62", "x": 3.51, "y": 26.67},
    63: {"name": "waypoint_63", "x": 32.33, "y": 21.88},
    64: {"name": "waypoint_64", "x": 28.22, "y": 21.88},
    65: {"name": "waypoint_65", "x": 23.34, "y": 21.88},
    66: {"name": "waypoint_66", "x": 17.98, "y": 21.88},
    67: {"name": "waypoint_67", "x": 12.25, "y": 21.88},
    68: {"name": "waypoint_68", "x": 9.29, "y": 21.88},
    69: {"name": "waypoint_69", "x": 3.51, "y": 21.88},
    70: {"name": "waypoint_70", "x": 34.42, "y": 17.98},
    71: {"name": "waypoint_71", "x": 28.22, "y": 17.98},
    72: {"name": "waypoint_72", "x": 23.34, "y": 17.98},
    73: {"name": "waypoint_73", "x": 17.98, "y": 17.98},
    74: {"name": "waypoint_74", "x": 12.25, "y": 17.98},
    75: {"name": "waypoint_75", "x": 17.98, "y": 8.79},
    76: {"name": "waypoint_76", "x": 20.26, "y": 13.38},
    77: {"name": "waypoint_77", "x": 17.98, "y": 13.38},
    78: {"name": "waypoint_78", "x": 15.65, "y": 13.38},
    79: {"name": "waypoint_79", "x": 32.33, "y": 8.79},
    80: {"name": "waypoint_80", "x": 17.98, "y": 4.16},
}


WAYPOINT_URDF = """<?xml version="1.0"?>
<robot name="waypoint">
  <link name="base_link">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
  </link>
</robot>
"""


class WaypointSpawner(Node):
    def __init__(self):
        super().__init__('waypoint_spawner')
        self.client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
        
    def spawn_waypoint(self, waypoint_id, x, y):
        req = SpawnEntity.Request()
        req.name = f"waypoint_{waypoint_id}"
        req.xml = WAYPOINT_URDF
        req.robot_namespace = ""
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.15)
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        req.initial_pose = pose
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Spawned waypoint_{waypoint_id} at ({x}, {y})")
        else:
            self.get_logger().error(f"Failed to spawn waypoint_{waypoint_id}")

    def spawn_all(self):
        self.get_logger().info("Spawning all waypoints...")
        
        for wp_id, data in WAREHOUSE_LOCATIONS.items():
            try:
                self.spawn_waypoint(wp_id, data['x'], data['y'])
            except Exception as e:
                self.get_logger().warn(f"Could not spawn waypoint {wp_id}: {e}")
        
        self.get_logger().info("Done spawning waypoints!")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSpawner()
    
    try:
        node.spawn_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
