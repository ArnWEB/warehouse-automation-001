#!/usr/bin/env python3
"""
Task Executor Node (Per Robot)
- Executes CuOpt plans using Nav2
- Handles plan updates and replanning
- Smart replan: doesn't restart if goal unchanged

State Machine:
  WAIT_FOR_PLAN → EXECUTE_GOAL → WAIT_NAV2 → NEXT_GOAL
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
import json
import time
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


class TaskExecutor(Node):
    def __init__(self, robot_id: str = "amr1", use_nav2: bool = False):
        super().__init__(f"task_executor_{robot_id}")

        self.robot_id = robot_id
        self.use_nav2 = use_nav2

        # Default positions matching cuopt_bridge ROBOTS config
        # amr1: charging_station (waypoint 80), amr2: staging (waypoint 75), amr3: asrs_output (waypoint 15)
        if robot_id == "amr1":
            self.current_x = 17.98
            self.current_y = 4.16
        elif robot_id == "amr2":
            self.current_x = 17.98
            self.current_y = 8.79
        elif robot_id == "amr3":
            self.current_x = 17.98
            self.current_y = 47.7
        else:
            self.current_x = 17.98
            self.current_y = 4.16
        self.current_theta = 0.0

        self.current_plan = []
        self.current_goal_index = 0
        self.plan_id = -1
        self.replan_requested = False
        self.executing = False

        self.plan_sub = self.create_subscription(
            String, "/fleet/cuopt_plan", self.plan_callback, 10
        )

        self.status_pub = self.create_publisher(Int32, f"/{robot_id}/status", 10)
        self.odom_pub = self.create_publisher(Odometry, f"/{robot_id}/odom", 10)
        self.debug_pub = self.create_publisher(
            String, f"/{robot_id}/executor_debug", 10
        )

        self.timer = self.create_timer(0.1, self.update_position)

        self.get_logger().info(f"Task Executor initialized for {robot_id}")

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            assignments = plan.get("assignments", {})

            # Use robot_id directly as key (e.g., "amr1", "amr2")
            if self.robot_id in assignments:
                new_tasks = assignments[self.robot_id]["tasks"]
                new_plan_id = plan["plan_id"]

                if new_plan_id != self.plan_id:
                    self.get_logger().info(
                        f"NEW PLAN {new_plan_id}: {self.robot_id} → {new_tasks}"
                    )

                    if self.executing:
                        self.handle_replan(new_tasks, new_plan_id)
                    else:
                        self.current_plan = new_tasks
                        self.current_goal_index = 0
                        self.plan_id = new_plan_id

                        self.execute_plan()
                else:
                    self.get_logger().debug(f"Same plan {new_plan_id}, ignoring")

        except Exception as e:
            self.get_logger().error(f"Failed to parse plan: {e}")

    def handle_replan(self, new_tasks, new_plan_id):
        current_goal = (
            self.current_plan[self.current_goal_index]
            if self.current_goal_index < len(self.current_plan)
            else -1
        )

        if current_goal in new_tasks[: self.current_goal_index + 1]:
            self.get_logger().info(f"Smart replan: continuing to goal {current_goal}")
            self.replan_requested = False
        else:
            self.get_logger().info(
                f"REPLAN: canceling goal {current_goal}, new route: {new_tasks}"
            )
            self.replan_requested = True
            self.current_plan = new_tasks
            self.current_goal_index = 0
            self.plan_id = new_plan_id

            self.execute_plan()

    def execute_plan(self):
        if not self.current_plan:
            self.get_logger().info("No tasks in plan, waiting...")
            self.executing = False
            self.publish_status(0)
            return

        self.executing = True
        self.publish_status(self.current_plan[self.current_goal_index])

        while self.current_goal_index < len(self.current_plan):
            task_id = self.current_plan[self.current_goal_index]

            self.get_logger().info(
                f"EXECUTING: {self.robot_id} → Task {task_id} @ {WAREHOUSE_LOCATIONS[task_id]['name']}"
            )

            self.publish_debug(
                {
                    "state": "moving",
                    "task_id": task_id,
                    "location": WAREHOUSE_LOCATIONS[task_id]["name"],
                    "progress": f"{self.current_goal_index + 1}/{len(self.current_plan)}",
                    "plan_id": self.plan_id,
                }
            )

            target = WAREHOUSE_LOCATIONS[task_id]
            self.move_to(target["x"], target["y"])

            self.current_goal_index += 1

            if self.replan_requested:
                self.get_logger().info(
                    "Replan received during execution, restarting..."
                )
                self.replan_requested = False
                break

        if self.current_goal_index >= len(self.current_plan):
            self.get_logger().info(
                f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
            )
            self.executing = False
            self.publish_status(0)

    def move_to(self, target_x, target_y):
        speed = 0.5
        arrived = False

        while not arrived:
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 0.1:
                arrived = True
                break

            angle = math.atan2(dy, dx)

            self.current_x += math.cos(angle) * speed * 0.1
            self.current_y += math.sin(angle) * speed * 0.1
            self.current_theta = angle

            time.sleep(0.1)

    def update_position(self):
        from geometry_msgs.msg import Quaternion

        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = self.robot_id

        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.current_theta / 2)
        q.w = math.cos(self.current_theta / 2)
        odom.pose.pose.orientation = q

        self.odom_pub.publish(odom)

    def publish_status(self, task_id: int):
        msg = Int32()
        msg.data = task_id
        self.status_pub.publish(msg)

    def publish_debug(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor(robot_id="amr1", use_nav2=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
