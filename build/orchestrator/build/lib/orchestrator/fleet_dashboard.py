#!/usr/bin/env python3
"""
Fleet Execution Visualizer
- Shows the complete flow: Tasks → Fleet Manager → CuOpt → Executors
- Displays in terminal like a graph
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import json
import time


WAREHOUSE_LOCATIONS = {
    0: {"name": "waypoint_0", "x": 17.98, "y": 61.49},
    1: {"name": "waypoint_1", "x": 23.34, "y": 58.46},
    2: {"name": "palletizer", "x": 20.26, "y": 58.46},
    3: {"name": "waypoint_3", "x": 15.65, "y": 58.46},
    4: {"name": "quality_check", "x": 17.98, "y": 58.46},
    5: {"name": "asrs_storage", "x": 12.25, "y": 58.46},
    6: {"name": "waypoint_6", "x": 32.33, "y": 53.34},
    7: {"name": "waypoint_7", "x": 26.77, "y": 53.34},
    8: {"name": "waypoint_8", "x": 23.34, "y": 53.34},
    9: {"name": "waypoint_9", "x": 17.98, "y": 53.34},
    10: {"name": "asrs_input", "x": 12.25, "y": 53.34},
    11: {"name": "waypoint_11", "x": 8.03, "y": 53.34},
    12: {"name": "waypoint_12", "x": 3.51, "y": 53.34},
    13: {"name": "waypoint_13", "x": 26.77, "y": 49.23},
    14: {"name": "waypoint_14", "x": 23.34, "y": 47.7},
    15: {"name": "asrs_output", "x": 17.98, "y": 47.7},
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
    42: {"name": "waypoint_42", "x": 24.72, "y": 35.29},
    43: {"name": "waypoint_43", "x": 17.98, "y": 35.29},
    44: {"name": "waypoint_44", "x": 11.41, "y": 35.29},
    45: {"name": "waypoint_45", "x": 6.19, "y": 35.29},
    46: {"name": "waypoint_46", "x": 3.51, "y": 35.29},
    47: {"name": "waypoint_47", "x": 1.48, "y": 35.29},
    48: {"name": "waypoint_48", "x": 6.19, "y": 32.0},
    49: {"name": "waypoint_49", "x": 11.41, "y": 32.0},
    50: {"name": "waypoint_50", "x": 17.98, "y": 32.0},
    51: {"name": "waypoint_51", "x": 24.72, "y": 32.0},
    52: {"name": "waypoint_52", "x": 30.2, "y": 32.0},
    53: {"name": "waypoint_53", "x": 6.19, "y": 27.69},
    54: {"name": "waypoint_54", "x": 11.41, "y": 27.69},
    55: {"name": "waypoint_55", "x": 17.98, "y": 27.69},
    56: {"name": "waypoint_56", "x": 24.72, "y": 27.69},
    57: {"name": "waypoint_57", "x": 30.2, "y": 27.69},
    58: {"name": "waypoint_58", "x": 6.19, "y": 22.51},
    59: {"name": "waypoint_59", "x": 11.41, "y": 22.51},
    60: {"name": "waypoint_60", "x": 17.98, "y": 22.51},
    61: {"name": "waypoint_61", "x": 24.72, "y": 22.51},
    62: {"name": "waypoint_62", "x": 30.2, "y": 22.51},
    63: {"name": "waypoint_63", "x": 6.19, "y": 17.17},
    64: {"name": "waypoint_64", "x": 11.41, "y": 17.17},
    65: {"name": "waypoint_65", "x": 17.98, "y": 17.17},
    66: {"name": "waypoint_66", "x": 24.72, "y": 17.17},
    67: {"name": "waypoint_67", "x": 30.2, "y": 17.17},
    68: {"name": "waypoint_68", "x": 6.19, "y": 12.47},
    69: {"name": "waypoint_69", "x": 11.41, "y": 12.47},
    70: {"name": "waypoint_70", "x": 17.98, "y": 12.47},
    71: {"name": "waypoint_71", "x": 24.72, "y": 12.47},
    72: {"name": "waypoint_72", "x": 30.2, "y": 12.47},
    73: {"name": "waypoint_73", "x": 6.19, "y": 8.79},
    74: {"name": "waypoint_74", "x": 11.41, "y": 8.79},
    75: {"name": "staging_area", "x": 17.98, "y": 8.79},
    76: {"name": "waypoint_76", "x": 24.72, "y": 8.79},
    77: {"name": "outbound_dock", "x": 30.2, "y": 8.79},
    78: {"name": "waypoint_78", "x": 6.19, "y": 4.16},
    79: {"name": "waypoint_79", "x": 11.41, "y": 4.16},
    80: {"name": "charging_station", "x": 17.98, "y": 4.16},
    81: {"name": "waypoint_81", "x": 24.72, "y": 4.16},
    82: {"name": "waypoint_82", "x": 30.2, "y": 4.16},
    83: {"name": "waypoint_83", "x": 34.42, "y": 24.85},
    84: {"name": "waypoint_84", "x": 32.33, "y": 24.85},
    85: {"name": "waypoint_85", "x": 34.42, "y": 20.21},
    86: {"name": "waypoint_86", "x": 32.33, "y": 20.21},
    87: {"name": "waypoint_87", "x": 3.51, "y": 24.91},
    88: {"name": "waypoint_88", "x": 1.48, "y": 24.91},
    89: {"name": "waypoint_89", "x": 3.51, "y": 20.27},
}


class FleetVisualizer(Node):
    def __init__(self):
        super().__init__("fleet_visualizer")

        self.get_logger().info("Fleet Visualizer starting...")
        self.get_logger().info("Subscribing to: /fleet/tasks, /fleet/cuopt_plan")

        self.tasks_sub = self.create_subscription(
            Int32MultiArray, "/fleet/tasks", self.tasks_callback, 10
        )

        self.plan_sub = self.create_subscription(
            String, "/fleet/cuopt_plan", self.plan_callback, 10
        )

        self.debug_subs = {}
        for robot_id in ["amr1", "amr2", "amr3"]:
            self.debug_subs[robot_id] = self.create_subscription(
                String,
                f"/{robot_id}/executor_debug",
                lambda msg, rid=robot_id: self.debug_callback(msg, rid),
                10,
            )

        self.plan_count = 0
        self.current_tasks = []
        self.robot_status = {r: "idle" for r in ["amr1", "amr2", "amr3"]}

        self.timer = self.create_timer(2.0, self.print_dashboard)

        self.get_logger().info("Fleet Visualizer started!")

    def tasks_callback(self, msg):
        self.current_tasks = list(msg.data)
        self.get_logger().info(f"📥 NEW TASKS: {self.current_tasks}")

    def plan_callback(self, msg):
        self.plan_count += 1
        plan = json.loads(msg.data)

        print("\n" + "=" * 60)
        print(f"📦 PLAN #{self.plan_count} - CuOpt Optimization Complete")
        print("=" * 60)

        for robot_id, assignment in plan.get("assignments", {}).items():
            tasks = assignment.get("tasks", [])
            task_names = [
                WAREHOUSE_LOCATIONS[t]["name"]
                for t in tasks
                if t in WAREHOUSE_LOCATIONS
            ]
            print(f"  🤖 {robot_id} → {task_names}")

        print("=" * 60 + "\n")

    def debug_callback(self, msg, robot_id):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            if state == "moving":
                loc = data.get("location", "")
                self.robot_status[robot_id] = f"moving to {loc}"
        except:
            pass

    def print_dashboard(self):
        print("\n" + "─" * 60)
        print("🚗 WAREHOUSE FLEET STATUS")
        print("─" * 60)

        print(f"📋 Pending Tasks: {self.current_tasks}")
        print()

        for robot_id in ["amr1", "amr2", "amr3"]:
            status = self.robot_status[robot_id]
            print(f"  🤖 {robot_id}: {status}")

        print("─" * 60)

        print("\n📍 Warehouse Locations:")
        for loc_id, loc in WAREHOUSE_LOCATIONS.items():
            marker = "⏳"
            if self.current_tasks and loc_id in self.current_tasks:
                marker = "📌"
            print(
                f"  {marker} {loc_id}: {loc['name']:15s} ({loc['x']:5.1f}, {loc['y']:5.1f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
