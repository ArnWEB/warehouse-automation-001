#!/usr/bin/env python3
"""
CuOpt Client Node
- Subscribes to /fleet/tasks and /fleet/robot_states
- Calls NVIDIA cuOpt (or mock) for optimization
- Publishes optimized plans to /fleet/cuopt_plan

This node converts ROS2 → cuOpt API.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, String
import json
import time
import random
import os

try:
    from cuopt_sh_client import CuOptServiceSelfHostClient

    CUOPT_SH_CLIENT_AVAILABLE = True
except ImportError:
    CUOPT_SH_CLIENT_AVAILABLE = False
    print("cuopt_sh_client not available, will use mock")

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
    90: {"name": "waypoint_90", "x": 1.48, "y": 20.27},
}

ZONE_TO_WAYPOINT = {
    "charging_station": 0,
    "inbound_dock": 1,
    "palletizer": 2,
    "quality_check": 3,
    "asrs_input": 4,
    "asrs_storage": 5,
    "asrs_output": 6,
    "staging_area": 7,
    "outbound_dock": 8,
}

STATIC_WAYPOINT_GRAPH = {
    "0": {
        "offsets": [
            0,
            3,
            5,
            10,
            15,
            19,
            21,
            24,
            27,
            33,
            41,
            47,
            50,
            54,
            59,
            63,
            66,
            70,
            75,
            79,
            82,
            85,
            90,
            95,
            97,
            99,
            103,
            108,
            109,
            116,
            119,
            122,
            127,
            129,
            135,
            139,
            145,
            150,
            154,
            155,
            156,
            160,
            164,
            169,
            175,
            180,
            184,
            188,
            189,
            191,
            193,
            195,
            197,
            199,
            202,
            204,
            206,
            208,
            211,
            216,
            220,
            224,
            228,
            233,
            236,
            238,
            240,
            242,
            245,
            247,
            249,
            251,
            253,
            255,
            259,
            264,
            268,
            273,
            277,
            279,
            283,
            288,
            292,
            294,
            295,
            299,
            300,
            304,
            308,
            309,
            313,
            314,
        ],
        "edges": [
            4,
            2,
            3,
            2,
            8,
            0,
            1,
            4,
            9,
            8,
            0,
            5,
            10,
            9,
            4,
            0,
            9,
            3,
            2,
            3,
            10,
            7,
            13,
            18,
            8,
            6,
            13,
            2,
            1,
            9,
            7,
            14,
            13,
            4,
            3,
            2,
            8,
            10,
            15,
            14,
            16,
            5,
            3,
            9,
            16,
            17,
            11,
            17,
            10,
            12,
            11,
            17,
            20,
            21,
            7,
            6,
            14,
            8,
            18,
            13,
            8,
            15,
            9,
            14,
            9,
            16,
            15,
            9,
            10,
            17,
            10,
            16,
            11,
            12,
            21,
            13,
            6,
            19,
            22,
            18,
            23,
            22,
            12,
            21,
            25,
            20,
            24,
            12,
            17,
            25,
            19,
            18,
            23,
            29,
            26,
            19,
            22,
            21,
            25,
            20,
            24,
            21,
            37,
            28,
            22,
            29,
            27,
            40,
            26,
            33,
            29,
            26,
            30,
            40,
            41,
            42,
            28,
            22,
            26,
            31,
            33,
            28,
            34,
            32,
            30,
            33,
            35,
            35,
            31,
            34,
            30,
            28,
            31,
            42,
            43,
            31,
            35,
            33,
            43,
            36,
            32,
            34,
            31,
            43,
            44,
            37,
            35,
            44,
            45,
            46,
            25,
            38,
            46,
            36,
            37,
            40,
            26,
            39,
            41,
            28,
            40,
            28,
            42,
            52,
            28,
            41,
            33,
            43,
            51,
            33,
            42,
            34,
            35,
            44,
            50,
            43,
            35,
            36,
            45,
            49,
            44,
            36,
            46,
            48,
            37,
            47,
            45,
            36,
            46,
            45,
            53,
            44,
            54,
            43,
            55,
            42,
            56,
            41,
            57,
            48,
            87,
            58,
            49,
            59,
            50,
            60,
            51,
            61,
            52,
            62,
            84,
            53,
            87,
            89,
            63,
            59,
            58,
            54,
            60,
            64,
            59,
            55,
            61,
            65,
            60,
            56,
            62,
            66,
            61,
            57,
            84,
            86,
            67,
            58,
            89,
            68,
            59,
            69,
            60,
            70,
            61,
            71,
            62,
            86,
            72,
            63,
            73,
            64,
            74,
            65,
            75,
            66,
            76,
            67,
            77,
            68,
            74,
            78,
            79,
            69,
            73,
            75,
            79,
            80,
            70,
            74,
            76,
            80,
            71,
            75,
            77,
            80,
            81,
            72,
            76,
            81,
            82,
            73,
            79,
            78,
            74,
            73,
            80,
            75,
            79,
            74,
            76,
            81,
            80,
            76,
            77,
            82,
            77,
            81,
            84,
            57,
            83,
            86,
            62,
            86,
            84,
            85,
            62,
            67,
            53,
            88,
            89,
            58,
            87,
            87,
            90,
            58,
            63,
            89,
        ],
        "weights": [
            3.03,
            3.79,
            3.82,
            3.08,
            5.12,
            3.79,
            3.08,
            2.28,
            5.60,
            5.98,
            3.82,
            3.40,
            6.15,
            5.63,
            2.33,
            3.03,
            5.12,
            2.33,
            2.28,
            3.40,
            5.12,
            5.56,
            6.91,
            7.51,
            3.43,
            5.56,
            4.11,
            5.98,
            5.12,
            5.36,
            3.43,
            5.64,
            5.35,
            5.12,
            5.63,
            5.60,
            5.36,
            5.73,
            5.64,
            7.78,
            8.04,
            5.12,
            6.15,
            5.73,
            5.64,
            6.47,
            4.22,
            4.75,
            4.22,
            4.52,
            4.52,
            6.44,
            7.44,
            8.01,
            4.11,
            6.91,
            3.76,
            5.35,
            6.52,
            3.76,
            5.64,
            5.36,
            7.78,
            5.36,
            5.64,
            5.73,
            5.73,
            8.04,
            5.64,
            4.48,
            6.47,
            4.48,
            4.75,
            6.44,
            3.02,
            6.52,
            7.51,
            2.88,
            1.80,
            2.88,
            1.80,
            3.40,
            7.44,
            2.97,
            1.94,
            2.97,
            1.94,
            8.01,
            3.02,
            3.55,
            3.40,
            1.80,
            2.88,
            4.24,
            4.09,
            1.80,
            2.88,
            1.94,
            2.97,
            1.94,
            2.97,
            3.55,
            4.02,
            4.11,
            4.09,
            3.92,
            2.09,
            4.65,
            2.09,
            4.88,
            1.83,
            4.11,
            5.58,
            6.21,
            5.05,
            5.82,
            1.83,
            4.24,
            3.92,
            5.36,
            2.71,
            5.58,
            2.71,
            5.73,
            5.36,
            6.01,
            6.34,
            2.71,
            5.73,
            5.36,
            2.71,
            4.88,
            6.01,
            4.85,
            7.10,
            2.71,
            5.73,
            5.36,
            4.65,
            2.96,
            2.71,
            5.73,
            6.34,
            7.38,
            4.73,
            5.78,
            2.96,
            5.11,
            5.59,
            7.42,
            4.02,
            2.03,
            4.65,
            5.78,
            2.03,
            2.09,
            4.65,
            2.09,
            2.13,
            6.21,
            2.13,
            5.05,
            5.48,
            3.29,
            5.82,
            5.48,
            4.85,
            6.74,
            3.29,
            7.10,
            6.74,
            4.65,
            7.38,
            6.57,
            3.29,
            6.57,
            4.73,
            5.11,
            5.22,
            3.29,
            5.22,
            5.59,
            2.68,
            3.29,
            4.65,
            2.03,
            2.68,
            7.42,
            2.03,
            3.29,
            4.31,
            3.29,
            4.31,
            3.29,
            4.31,
            3.29,
            4.31,
            3.29,
            4.31,
            4.31,
            3.86,
            5.18,
            4.31,
            5.18,
            4.31,
            5.18,
            4.31,
            5.18,
            4.31,
            5.18,
            3.55,
            5.18,
            3.60,
            3.49,
            5.34,
            5.22,
            5.22,
            5.18,
            6.57,
            5.34,
            6.57,
            5.18,
            6.74,
            5.34,
            6.74,
            5.18,
            5.48,
            5.34,
            5.48,
            5.18,
            3.16,
            3.13,
            5.34,
            5.34,
            4.10,
            4.70,
            5.34,
            4.70,
            5.34,
            4.70,
            5.34,
            4.70,
            5.34,
            3.71,
            4.70,
            4.70,
            3.68,
            4.70,
            3.68,
            4.70,
            3.68,
            4.70,
            3.68,
            4.70,
            3.68,
            4.70,
            3.68,
            5.22,
            4.63,
            6.98,
            3.68,
            5.22,
            6.57,
            4.63,
            8.04,
            3.68,
            6.57,
            6.74,
            4.63,
            3.68,
            6.74,
            5.48,
            8.18,
            4.63,
            3.68,
            5.48,
            7.17,
            4.63,
            4.63,
            5.22,
            5.22,
            4.63,
            6.98,
            6.57,
            4.63,
            6.57,
            8.04,
            8.18,
            6.74,
            6.74,
            4.63,
            7.17,
            5.48,
            4.63,
            5.48,
            2.09,
            3.55,
            2.09,
            4.64,
            3.16,
            2.09,
            4.64,
            2.09,
            3.13,
            3.71,
            3.86,
            2.03,
            4.64,
            3.60,
            2.03,
            4.64,
            2.03,
            3.49,
            4.10,
            2.03,
        ],
    }
}

CUOPT_SERVER_IP = "43.201.55.122"
CUOPT_SERVER_PORT = 5000


class CuOptClient(Node):
    def __init__(self, use_mock: bool = True):
        super().__init__("cuopt_client")

        self.use_mock = use_mock or not CUOPT_SH_CLIENT_AVAILABLE

        if self.use_mock:
            self.get_logger().info("CuOpt Client: Using MOCK solver")
        else:
            server = f"{CUOPT_SERVER_IP}:{CUOPT_SERVER_PORT}"
            self.get_logger().info(f"CuOpt Client: Using REAL cuOpt API at {server}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.task_sub = self.create_subscription(
            Int32MultiArray, "/cuopt/trigger", self.tasks_callback, qos
        )

        self.robot_state_sub = self.create_subscription(
            String, "/fleet/robot_states", self.robot_states_callback, qos
        )

        self.plan_pub = self.create_publisher(String, "/fleet/cuopt_plan", qos)

        self.plan_pub_debug = self.create_publisher(
            String, "/fleet/cuopt_plan_debug", 10
        )

        self.pending_tasks = []
        self.robot_states = {}
        self.plan_count = 0

        self.get_logger().info("CuOpt Client Node initialized")

    def robot_states_callback(self, msg):
        try:
            states = json.loads(msg.data)
            self.robot_states = {r["robot_id"]: r for r in states}
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot states: {e}")

    def tasks_callback(self, msg):
        task_ids = list(msg.data)
        if task_ids:
            self.pending_tasks.extend(task_ids)
            self.get_logger().info(f"Received {len(task_ids)} new tasks: {task_ids}")

            if len(self.pending_tasks) >= 1:
                if not self.robot_states:
                    self.robot_states = {
                        "amr1": {"robot_id": "amr1", "x": 0.0, "y": 0.0},
                        "amr2": {"robot_id": "amr2", "x": 0.0, "y": 0.0},
                    }
                    self.get_logger().info("Using default robot positions")
                self.solve_and_publish()

    def solve_and_publish(self):
        if not self.robot_states:
            self.get_logger().warn("No robot states available, cannot optimize")
            return

        self.plan_count += 1

        if self.use_mock:
            plan = self._solve_mock()
        else:
            plan = self._solve_cuopt()

        msg = String()
        msg.data = json.dumps(plan)
        self.plan_pub.publish(msg)

        debug_msg = String()
        debug_msg.data = json.dumps(
            {
                "plan_id": plan["plan_id"],
                "robot_assignments": {
                    k: v["tasks"] for k, v in plan["assignments"].items()
                },
                "timestamp": time.time(),
            }
        )
        self.plan_pub_debug.publish(debug_msg)

        self.get_logger().info(
            f"Plan {plan['plan_id']}: "
            + ", ".join([f"{k}→{v['tasks']}" for k, v in plan["assignments"].items()])
        )

        self.pending_tasks = []

    def _solve_mock(self):
        n_robots = len(self.robot_states)
        n_tasks = len(self.pending_tasks)

        assignments = {}
        tasks_per_robot = [[] for _ in range(n_robots)]

        for i, task_id in enumerate(self.pending_tasks):
            robot_idx = i % n_robots
            tasks_per_robot[robot_idx].append(task_id)

        for idx, (robot_id, state) in enumerate(self.robot_states.items()):
            assignments[robot_id] = {
                "tasks": tasks_per_robot[idx],
                "total_cost": random.uniform(10, 50),
                "estimated_time": len(tasks_per_robot[idx]) * 2.5,
            }

        return {
            "plan_id": self.plan_count,
            "created_at": time.time(),
            "solver": "mock" if self.use_mock else "cuopt",
            "solve_time_ms": random.uniform(50, 200),
            "assignments": assignments,
            "total_cost": sum(a["total_cost"] for a in assignments.values()),
            "num_tasks": n_tasks,
        }

    def _repoll(self, cuopt_service_client, solution, repoll_tries=500):
        if "reqId" in solution and "response" not in solution:
            req_id = solution["reqId"]
            for i in range(repoll_tries):
                solution = cuopt_service_client.repoll(req_id, response_type="dict")
                if "reqId" in solution and "response" in solution:
                    break

                time.sleep(1)

        return solution

    def _build_waypoint_graph(self):
        json_path = "/tmp/cuopt_waypoint_graph.json"

        if os.path.exists(json_path):
            try:
                with open(json_path, "r") as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to load waypoint graph from {json_path}: {e}"
                )

        return STATIC_WAYPOINT_GRAPH

    def _find_nearest_waypoint(self, x, y):
        min_dist = float("inf")
        nearest_wp = 0
        for wp_id, loc in WAREHOUSE_LOCATIONS.items():
            dist = ((loc["x"] - x) ** 2 + (loc["y"] - y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                nearest_wp = wp_id
        return nearest_wp

    def _convert_robot_states_to_fleet(self):
        vehicle_locations = []

        for robot_id, state in self.robot_states.items():
            x = state.get("x", 0.0)
            y = state.get("y", 0.0)

            # Default robot at (0,0) should map to waypoint 0 (depot)
            if x == 0.0 and y == 0.0:
                nearest_wp = 0
            else:
                nearest_wp = self._find_nearest_waypoint(x, y)

            vehicle_locations.append([nearest_wp, 0])

        n_robots = len(self.robot_states)
        capacities = [[11] * n_robots]

        return {
            "vehicle_locations": vehicle_locations,
            "capacities": capacities,
            "vehicle_time_windows": None,
        }

    def _convert_tasks_to_taskdata(self):
        task_locations = []

        for task_id in self.pending_tasks:
            task_locations.append(task_id)

        demand = [[1] * len(self.pending_tasks)]

        return {
            "task_locations": task_locations,
            "demand": demand,
            "task_time_windows": None,
            "service_times": None,
        }

    def _solve_cuopt(self):
        try:
            waypoint_graph = self._build_waypoint_graph()
            fleet_data = self._convert_robot_states_to_fleet()
            task_data = self._convert_tasks_to_taskdata()

            data = {
                "cost_waypoint_graph_data": {"waypoint_graph": waypoint_graph},
                "fleet_data": fleet_data,
                "task_data": task_data,
                "solver_config": {"time_limit": 0.01},
            }

            with open("/tmp/cuopt_payload.json", "w") as f:
                json.dump(data, f, indent=2)
            print(f"\nPayload written to /tmp/cuopt_payload.json")
            print("Use: cat /tmp/cuopt_payload.json to view")

            self.get_logger().info("=== DATA SENT TO CUOPT ===")
            self.get_logger().info(f"Fleet Data: {json.dumps(fleet_data, indent=2)}")
            self.get_logger().info(f"Task Data: {json.dumps(task_data, indent=2)}")
            self.get_logger().info(
                f"Waypoint Graph: {json.dumps(waypoint_graph, indent=2)}"
            )
            self.get_logger().info("==========================")

            self.get_logger().info(
                f"Connecting to CuOpt server at {CUOPT_SERVER_IP}:{CUOPT_SERVER_PORT}"
            )

            cuopt_service_client = CuOptServiceSelfHostClient(
                ip=CUOPT_SERVER_IP,
                port=CUOPT_SERVER_PORT,
                polling_timeout=25,
                timeout_exception=False,
            )

            self.get_logger().info("Sending optimization request to CuOpt...")

            solution = cuopt_service_client.get_optimized_routes(data)

            self.get_logger().info("Waiting for CuOpt response...")
            solution = self._repoll(cuopt_service_client, solution)

            self.get_logger().info("=== CUOPT RESPONSE ===")
            self.get_logger().info(f"{json.dumps(solution, indent=2)}")
            self.get_logger().info("========================")

            response = solution.get("response", {})
            solver_response = response.get("solver_response", {})

            status = solver_response.get("status", -1)
            if status != 0:
                self.get_logger().warn(f"CuOpt returned non-zero status: {status}")

            vehicle_data = solver_response.get("vehicle_data", {})
            solution_cost = solver_response.get("solution_cost", 0.0)
            num_vehicles = solver_response.get("num_vehicles", 1)

            self.get_logger().info(
                f"CuOpt optimization complete: {num_vehicles} vehicles, "
                f"cost: {solution_cost:.2f}"
            )

            assignments = {}
            robot_ids = list(self.robot_states.keys())

            for robot_idx_str, robot_data in vehicle_data.items():
                robot_idx = int(robot_idx_str)
                if robot_idx < len(robot_ids):
                    robot_id = robot_ids[robot_idx]
                    task_id_list = robot_data.get("task_id", [])
                    route = robot_data.get("route", [])

                    task_ids = []
                    for t in route:
                        if t > 0 and (t - 1) < len(self.pending_tasks):
                            task_ids.append(self.pending_tasks[t - 1])

                    assignments[robot_id] = {
                        "tasks": task_ids,
                        "total_cost": solution_cost / max(num_vehicles, 1),
                        "estimated_time": len(task_ids) * 2.5,
                    }

            solve_time_ms = solver_response.get("solve_time_ms", 0.0)

            return {
                "plan_id": self.plan_count,
                "created_at": time.time(),
                "solver": "cuopt",
                "solve_time_ms": solve_time_ms,
                "assignments": assignments,
                "total_cost": solution_cost,
                "num_tasks": len(self.pending_tasks),
            }

        except ConnectionError as e:
            self.get_logger().error(
                f"Failed to connect to CuOpt server at "
                f"{CUOPT_SERVER_IP}:{CUOPT_SERVER_PORT}: {e}"
            )
            self.get_logger().info("Falling back to mock solver")
            return self._solve_mock()

        except TimeoutError as e:
            self.get_logger().error(f"CuOpt request timed out: {e}")
            self.get_logger().info("Falling back to mock solver")
            return self._solve_mock()

        except Exception as e:
            self.get_logger().error(f"CuOpt solver failed: {type(e).__name__}: {e}")
            self.get_logger().info("Falling back to mock solver")
            return self._solve_mock()


def main(args=None):
    rclpy.init(args=args)
    node = CuOptClient(use_mock=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
