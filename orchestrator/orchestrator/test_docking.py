#!/usr/bin/env python3
"""
Basic Nav2 Docking Test Script
Usage:
    ros2 run orchestrator test_docking
    or
    ros2 run orchestrator test_docking --ros-args -p namespace:=amr1
    or
    python3 test_docking.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot
from geometry_msgs.msg import PoseStamped
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import math


class TestDocking(Node):
    def __init__(self):
        super().__init__("test_docking")

        # Declare parameter for namespace
        if not self.has_parameter("namespace"):
            self.declare_parameter("namespace", "amr1")
        namespace = self.get_parameter("namespace").value

        # Create docking action client with namespace
        action_topic = f"/{namespace}/dock_robot"
        self.docking_client = ActionClient(self, DockRobot, action_topic)

        # Create lifecycle clients for configuring/activating docking server
        self._change_state_client = self.create_client(
            lifecycle_msgs.srv.ChangeState, f"/{namespace}/docking_server/change_state"
        )

        self.get_logger().info(
            f"TestDocking node started, using action: {action_topic}"
        )

    def activate_docking_server(self):
        """Configure and activate the docking server."""
        from lifecycle_msgs.srv import ChangeState
        from lifecycle_msgs.msg import Transition

        self.get_logger().info("Configuring docking server...")

        # Configure
        req = ChangeState.Request()
        req.transition = Transition(id=Transition.TRANSITION_CONFIGURE)

        if self._change_state_client.wait_for_service(timeout_sec=5.0):
            future = self._change_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            self.get_logger().info("Docking server configured")
        else:
            self.get_logger().warn(
                "Could not configure docking server - may already be configured"
            )

        # Activate
        self.get_logger().info("Activating docking server...")
        req.transition = Transition(id=Transition.TRANSITION_ACTIVATE)

        if self._change_state_client.wait_for_service(timeout_sec=5.0):
            future = self._change_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            self.get_logger().info("Docking server activated!")
        else:
            self.get_logger().error("Could not activate docking server!")
            return False

        return True

    def dock_by_id(self, dock_id: str = "dock_0"):
        """Send docking goal using dock pose directly."""

        # First try to activate the docking server
        self.activate_docking_server()

        self.get_logger().info(f"Waiting for docking server...")

        # Wait for server - increase timeout
        self.get_logger().info("Waiting for docking server (30s timeout)...")
        if not self.docking_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Docking server not available!")
            return False

        self.get_logger().info("Docking server available!")

        # Use dock pose directly instead of dock_id
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False  # Use pose instead of ID

        # Dock 0: x=13.15, y=17.03, theta=90deg (1.57 rad)
        # Dock 1: x=11.88591, y=15.9442, theta=90deg (1.57 rad)
        dock_poses = {
            "dock_0": {"x": 13.15, "y": 17.03, "theta": 1.57},
            "dock_1": {"x": 11.88591, "y": 15.9442, "theta": 1.57},
        }

        dock_pose_data = dock_poses.get(dock_id, dock_poses["dock_0"])

        # Create dock pose
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = "map"
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        dock_pose.pose.position.x = dock_pose_data["x"]
        dock_pose.pose.position.y = dock_pose_data["y"]
        dock_pose.pose.position.z = 0.0
        dock_pose.pose.orientation.z = math.sin(dock_pose_data["theta"] / 2)
        dock_pose.pose.orientation.w = math.cos(dock_pose_data["theta"] / 2)

        goal_msg.dock_pose = dock_pose
        # Don't set dock_type - let server figure it out
        goal_msg.max_staging_time = 30.0
        goal_msg.navigate_to_staging_pose = True

        # Debug: print goal message
        self.get_logger().info(
            f"Goal msg: use_dock_id={goal_msg.use_dock_id}, dock_type='{goal_msg.dock_type}'"
        )

        self.get_logger().info(
            f"Sending dock goal: pose=({dock_pose_data['x']}, {dock_pose_data['y']}), "
            f"theta={math.degrees(dock_pose_data['theta']):.0f}deg"
        )

        # Send goal
        self._send_goal_future = self.docking_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected!")
            # Try to get more info about rejection
            self.get_logger().error(f"Goal handle result: {goal_handle}")
            return

        self.get_logger().info("Dock goal accepted!")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle goal result."""
        result = future.result().result

        if result.error_code_id == 0:
            self.get_logger().info("Docking SUCCESSFUL!")
        else:
            self.get_logger().error(
                f"Docking FAILED with error code: {result.error_code_id}"
            )

        # Shutdown after result
        self.get_logger().info("Shutting down...")
        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)

    node = TestDocking()

    import sys

    dock_id = "dock_0"
    if len(sys.argv) > 1:
        dock_id = sys.argv[1]

    try:
        node.dock_by_id(dock_id)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
