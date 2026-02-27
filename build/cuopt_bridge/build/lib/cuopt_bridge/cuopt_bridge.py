#!/usr/bin/env python3
"""
CuOpt Mock Bridge (NVIDIA CuOpt Simulation)
- Subscribes to /orders
- Creates optimized task plan for robot fleet
- Publishes plan to /cuopt/plan

Real CuOpt features simulated:
- Task dependencies
- Time windows
- Robot capabilities
- Cost optimization
- Multi-objective routing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import json
import time
import random

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
    75: {"name": "waypoint_75", "x": 17.98, "y": 8.79},
    76: {"name": "waypoint_76", "x": 24.72, "y": 8.79},
    77: {"name": "waypoint_77", "x": 30.2, "y": 8.79},
    78: {"name": "waypoint_78", "x": 6.19, "y": 4.16},
    79: {"name": "waypoint_79", "x": 11.41, "y": 4.16},
    80: {"name": "waypoint_80", "x": 17.98, "y": 4.16},
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

LOCATIONS = {
    "inbound_dock": {"x": 17.98, "y": 61.49, "zone": "Z1"},
    "palletizer": {"x": 20.26, "y": 58.46, "zone": "Z1"},
    "quality_check": {"x": 17.98, "y": 58.46, "zone": "Z2"},
    "asrs_input": {"x": 12.25, "y": 53.34, "zone": "Z2"},
    "asrs_storage": {"x": 12.25, "y": 58.46, "zone": "Z3"},
    "asrs_output": {"x": 17.98, "y": 47.7, "zone": "Z4"},
    "staging_area": {"x": 17.98, "y": 8.79, "zone": "Z4"},
    "outbound_dock": {"x": 30.2, "y": 8.79, "zone": "Z5"},
    "charging_station": {"x": 17.98, "y": 4.16, "zone": "Z0"},
}

ZONE_TO_WAYPOINT = {
    "charging_station": 80,
    "inbound_dock": 0,
    "palletizer": 2,
    "quality_check": 4,
    "asrs_input": 10,
    "asrs_storage": 5,
    "asrs_output": 15,
    "staging_area": 75,
    "outbound_dock": 77,
}

# Robot fleet with capabilities
ROBOTS = {
    "amr1": {
        "type": "forklift",
        "capacity_kg": 1000,
        "max_runtime_min": 480,
        "current_battery": 85,
        "current_loc": "charging_station",
        "capabilities": ["pickup", "transport", "deliver", "stack"],
    },
    "amr2": {
        "type": "forklift",
        "capacity_kg": 1000,
        "max_runtime_min": 480,
        "current_battery": 92,
        "current_loc": "staging_area",
        "capabilities": ["pickup", "transport", "deliver", "stack"],
    },
    "amr3": {
        "type": "conveyor",
        "capacity_kg": 500,
        "max_runtime_min": 480,
        "current_battery": 78,
        "current_loc": "asrs_output",
        "capabilities": ["pickup", "transport", "deliver"],
    },
}

# Task types
TASK_TYPES = ["pickup", "transport", "deliver", "stack", "unstack", "recharge"]


class CuOptBridge(Node):
    def __init__(self):
        super().__init__("cuopt_bridge")

        # Subscribe to orders
        self.order_sub = self.create_subscription(
            String, "/orders", self.order_callback, 10
        )

        # Subscribe to robot states
        self.robot_state_sub = self.create_subscription(
            String, "/fleet/robot_states", self.robot_states_callback, 10
        )

        # Publish optimized plan
        self.plan_pub = self.create_publisher(String, "/cuopt/plan", 10)

        # Publish trigger for cuopt_client
        self.trigger_pub = self.create_publisher(Int32MultiArray, "/cuopt/trigger", 10)

        # Publish fleet status
        self.fleet_pub = self.create_publisher(String, "/fleet/status", 10)

        # Publish robot states for cuopt_client
        self.robot_states_pub = self.create_publisher(String, "/fleet/robot_states", 10)

        self.plan_count = 0
        self.robot_states = {}

        # Publish robot states periodically
        self.robot_states_timer = self.create_timer(1.0, self.publish_fleet_status)

        # Publish initial fleet status
        self.publish_fleet_status()

        self.get_logger().info("CuOpt Bridge Started (NVIDIA CuOpt Mock)")
        self.get_logger().info(f"Available robots: {list(ROBOTS.keys())}")
        self.get_logger().info(f"Warehouse zones: {len(LOCATIONS)} locations")

    def robot_states_callback(self, msg):
        try:
            states = json.loads(msg.data)
            self.robot_states = {r["robot_id"]: r for r in states}
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot states: {e}")

    def publish_fleet_status(self):
        """Publish current fleet status"""
        status = {
            "timestamp": time.time(),
            "robots": ROBOTS,
            "total_active": sum(
                1 for r in ROBOTS.values() if r["current_battery"] > 20
            ),
            "total_charging": sum(
                1 for r in ROBOTS.values() if r["current_battery"] <= 20
            ),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.fleet_pub.publish(msg)

        # Also publish robot states for cuopt_client
        robot_states_list = []
        for robot_id, robot_info in ROBOTS.items():
            loc_name = robot_info.get("current_loc", "charging_station")
            loc = LOCATIONS.get(loc_name, LOCATIONS["charging_station"])
            robot_states_list.append(
                {
                    "robot_id": robot_id,
                    "x": loc["x"],
                    "y": loc["y"],
                    "battery": robot_info["current_battery"],
                    "capacity_kg": robot_info["capacity_kg"],
                }
            )

        robot_states_msg = String()
        robot_states_msg.data = json.dumps(robot_states_list)
        self.robot_states_pub.publish(robot_states_msg)

    def order_callback(self, msg):
        try:
            order = json.loads(msg.data)
            self.get_logger().info(
                f"Received order: {order['order_id']} (priority: {order['priority']})"
            )

            # Create optimized plan (simulating CuOpt)
            plan = self.create_optimized_plan(order)

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Extract waypoint IDs from tasks and publish to /cuopt/trigger
            task_waypoint_ids = []
            for task in plan.get("tasks", []):
                location = task.get("location", "")
                # Map location name to waypoint ID using ZONE_TO_WAYPOINT
                if location in ZONE_TO_WAYPOINT:
                    task_waypoint_ids.append(ZONE_TO_WAYPOINT[location])

            if task_waypoint_ids:
                trigger_msg = Int32MultiArray()
                trigger_msg.data = task_waypoint_ids
                self.trigger_pub.publish(trigger_msg)
                self.get_logger().info(
                    f"Published trigger to cuopt_client: {task_waypoint_ids}"
                )

            self.get_logger().info(
                f"Published optimized plan: {plan['plan_id']} "
                f"(cost: ${plan['total_cost']:.2f}, "
                f"est. time: {plan['route']['estimated_duration_min']}min)"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to process order: {e}")

    def create_optimized_plan(self, order):
        """
        Create an optimized plan simulating NVIDIA CuOpt
        CuOpt considers: travel time, robot availability, task dependencies,
        time windows, robot capabilities, battery constraints
        """
        self.plan_count += 1
        plan_id = f"PLAN-{self.plan_count:03d}"

        # Select best robot based on CuOpt-style optimization
        selected_robot = self.select_optimal_robot(order)

        # Calculate route based on locations
        tasks = self.optimize_task_sequence(order, selected_robot)

        # Calculate metrics
        total_distance = self.calculate_route_distance(tasks)
        estimated_duration = len(tasks) * 2.5  # 2.5 min per task average

        return {
            "plan_id": plan_id,
            "order_id": order["order_id"],
            "created_at": time.time(),
            # Optimization metadata
            "optimizer": "NVIDIA CuOpt Mock v1.0",
            "optimization_objective": "minimize_total_cost",
            "solver_time_ms": random.randint(50, 200),
            # Selected robot
            "assigned_robot": {
                "robot_id": selected_robot,
                "type": ROBOTS[selected_robot]["type"],
                "battery_before": ROBOTS[selected_robot]["current_battery"],
            },
            # Task sequence (optimized)
            "tasks": tasks,
            # Route metrics
            "route": {
                "total_distance_m": total_distance,
                "estimated_duration_min": estimated_duration,
                "num_stops": len(tasks),
                "zones_visited": list(set(t["zone"] for t in tasks)),
            },
            # Cost metrics (CuOpt-style)
            "cost_breakdown": {
                "travel_cost": total_distance * 0.10,
                "labor_cost": estimated_duration * 0.50,
                "energy_cost": total_distance * 0.05,
                "priority_penalty": (6 - order["priority"]) * 5.0,
            },
            "total_cost": (total_distance * 0.10)
            + (estimated_duration * 0.50)
            + (total_distance * 0.05)
            + ((6 - order["priority"]) * 5.0),
            # Constraints satisfied
            "constraints": {
                "time_window_satisfied": True,
                "capacity_satisfied": True,
                "battery_satisfied": True,
                "capability_satisfied": True,
            },
            "status": "optimized",
        }

    def select_optimal_robot(self, order):
        """
        Simulate CuOpt robot selection based on:
        - Battery level (prefer higher)
        - Proximity to task location
        - Capability match
        - Current workload
        """
        # Simple selection: pick robot with highest battery that's available
        available = [r for r, info in ROBOTS.items() if info["current_battery"] > 20]

        if not available:
            return "amr1"  # Fallback

        # Sort by battery (descending)
        available.sort(key=lambda r: ROBOTS[r]["current_battery"], reverse=True)

        return available[0]

    def optimize_task_sequence(self, order, robot_id):
        """
        Optimize task sequence simulating CuOpt routing
        Returns tasks with time windows and dependencies
        """
        tasks = []
        task_id_base = f"{robot_id}-{self.plan_count}"

        # Task 1: Pickup from palletizer
        tasks.append(
            {
                "task_id": f"{task_id_base}-P001",
                "task_type": "pickup",
                "location": "palletizer",
                "zone": "Z1",
                "coords": LOCATIONS["palletizer"],
                "order_id": order["order_id"],
                "items": order["items"],
                "quantities": order["quantities"],
                "weight_kg": sum(order["quantities"]),
                "sequence": 1,
                "dependency": None,
                "time_window": {
                    "earliest": time.time() + 60,
                    "latest": time.time() + 300,
                },
                "service_time_min": 2.0,
            }
        )

        # Task 2: Transport to ASRS input
        tasks.append(
            {
                "task_id": f"{task_id_base}-T001",
                "task_type": "transport",
                "location": "asrs_input",
                "zone": "Z2",
                "coords": LOCATIONS["asrs_input"],
                "order_id": order["order_id"],
                "sequence": 2,
                "dependency": f"{task_id_base}-P001",
                "time_window": {
                    "earliest": time.time() + 180,
                    "latest": time.time() + 420,
                },
                "service_time_min": 1.0,
            }
        )

        # Task 3: Store in ASRS
        tasks.append(
            {
                "task_id": f"{task_id_base}-S001",
                "task_type": "stack",
                "location": "asrs_storage",
                "zone": "Z3",
                "coords": LOCATIONS["asrs_storage"],
                "order_id": order["order_id"],
                "sequence": 3,
                "dependency": f"{task_id_base}-T001",
                "time_window": {
                    "earliest": time.time() + 300,
                    "latest": time.time() + 600,
                },
                "service_time_min": 3.0,
                "asrs_bin": f"BIN-{random.randint(1, 50):03d}",
            }
        )

        return tasks

    def calculate_route_distance(self, tasks):
        """Calculate total route distance"""
        total = 0.0
        prev_loc = ROBOTS["amr1"]["current_loc"]  # Start from robot location

        for task in tasks:
            curr_loc = task["location"]
            if prev_loc in LOCATIONS and curr_loc in LOCATIONS:
                dx = LOCATIONS[curr_loc]["x"] - LOCATIONS[prev_loc]["x"]
                dy = LOCATIONS[curr_loc]["y"] - LOCATIONS[prev_loc]["y"]
                total += (dx**2 + dy**2) ** 0.5
            prev_loc = curr_loc

        return round(total, 2)


def main(args=None):
    rclpy.init(args=args)
    node = CuOptBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
