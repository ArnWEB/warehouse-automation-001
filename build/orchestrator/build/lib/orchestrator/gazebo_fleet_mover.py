#!/usr/bin/env python3
"""
Gazebo Fleet Mover - Simple version
- Subscribes to /fleet/cuopt_plan directly
- Moves Gazebo robots to their assigned task locations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import json
import math


WAREHOUSE_LOCATIONS = {
    0: {"name": "charging", "x": -5.0, "y": 0.0},
    1: {"name": "palletizer", "x": -8.0, "y": 3.0},
    2: {"name": "quality_check", "x": -4.0, "y": 3.0},
    3: {"name": "asrs_in", "x": 8.0, "y": 3.0},
    4: {"name": "asrs_store", "x": 8.0, "y": 5.0},
    5: {"name": "asrs_out", "x": 8.0, "y": 7.0},
    6: {"name": "staging", "x": 0.0, "y": -5.0},
    7: {"name": "outbound", "x": 5.0, "y": -5.0},
}

ROBOT_START_POSITIONS = {
    "amr1": {"x": -5.0, "y": 0.0},
    "amr2": {"x": -5.0, "y": 2.0},
    "amr3": {"x": -5.0, "y": -2.0},
}


class GazeboFleetMover(Node):
    def __init__(self):
        super().__init__("gazebo_fleet_mover")

        self.get_logger().info("Gazebo Fleet Mover starting...")

        self.plan_sub = self.create_subscription(
            String, "/fleet/cuopt_plan", self.plan_callback, 10
        )

        self.get_logger().info("Waiting for /gazebo/set_model_state service...")

        self.set_model_client = self.create_client(
            SetModelState, "/gazebo/set_model_state"
        )

        if self.set_model_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("✅ Gazebo service connected!")
        else:
            self.get_logger().warn(
                "❌ Gazebo service not available - robots will not move"
            )

        self.robot_positions = dict(ROBOT_START_POSITIONS)
        self.robot_targets = {k: None for k in ROBOT_START_POSITIONS}

        self.timer = self.create_timer(0.2, self.move_robots)

        self.get_logger().info("Gazebo Fleet Mover initialized")
        self.get_logger().warn("To enable robot movement, install gazebo_ros2_control")

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            assignments = plan.get("assignments", {})

            for robot_id, assignment in assignments.items():
                tasks = assignment.get("tasks", [])
                if tasks:
                    first_task = tasks[0]
                    if first_task in WAREHOUSE_LOCATIONS:
                        target = WAREHOUSE_LOCATIONS[first_task]
                        self.robot_targets[robot_id] = {
                            "x": target["x"],
                            "y": target["y"],
                        }
                        self.get_logger().info(
                            f"{robot_id} assigned to: {target['name']}"
                        )
        except Exception as e:
            self.get_logger().error(f"Failed to parse plan: {e}")

    def move_robots(self):
        for robot_id, target in self.robot_targets.items():
            if target:
                current = self.robot_positions[robot_id]
                dx = target["x"] - current["x"]
                dy = target["y"] - current["y"]
                dist = math.sqrt(dx * dx + dy * dy)

                speed = 0.5
                if dist > 0.1:
                    current["x"] += (dx / dist) * speed
                    current["y"] += (dy / dist) * speed
                else:
                    current["x"] = target["x"]
                    current["y"] = target["y"]

                self._set_model_state(robot_id, current["x"], current["y"])

    def _set_model_state(self, model_name, x, y):
        req = SetModelState.Request()

        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.1
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0

        req.model_state = model_state

        try:
            future = self.set_model_client.call_async(req)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFleetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
