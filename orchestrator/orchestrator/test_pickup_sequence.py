#!/usr/bin/env python3
"""
Test script to send CuOpt plan for pickup sequence [0, 1]
Usage:
    ros2 run orchestrator test_pickup_sequence
    or
    python3 test_pickup_sequence.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TestPickup(Node):
    def __init__(self):
        super().__init__("test_pickup")
        self.plan_pub = self.create_publisher(String, "/fleet/cuopt_plan", 10)
        self.get_logger().info("TestPickup node started")

    def send_plan(self, robot_id: str = "amr1"):
        plan = {"plan_id": 1, "assignments": {robot_id: {"route": [0, 1]}}}

        msg = String()
        msg.data = json.dumps(plan)

        self.get_logger().info(f"Sending plan: {plan}")
        self.plan_pub.publish(msg)
        self.get_logger().info(f"Plan sent for {robot_id} with route [0, 1]")


def main(args=None):
    rclpy.init(args=args)

    node = TestPickup()

    # Wait for subscribers
    time.sleep(1.0)

    # Send plan
    node.send_plan("amr1")

    # Keep node alive for a bit to ensure message is sent
    time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
