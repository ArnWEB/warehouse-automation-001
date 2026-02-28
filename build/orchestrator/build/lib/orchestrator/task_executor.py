#!/usr/bin/env python3
"""
Task Executor Node (Per Robot)
- Executes CuOpt plans using cmd_vel OR Nav2 (Isaac Sim compatible)
- Handles plan updates and replanning
- Smart replan: doesn't restart if goal unchanged

State Machine:
  WAIT_FOR_PLAN → TURNING → MOVING → ARRIVED → NEXT_GOAL

Usage:
  cmd_vel mode: ros2 run orchestrator task_executor
  Nav2 mode: ros2 run orchestrator task_executor --ros-args -p use_nav2:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
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
    # Movement parameters for iw_hub AMR
    LINEAR_SPEED = 0.5  # m/s
    ANGULAR_SPEED = 2.0  # rad/s
    ARRIVAL_THRESHOLD = 0.2  # meters (increased for Isaac Sim)
    TURN_THRESHOLD = 0.3  # radians (increased - ~17 degrees)

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

        # Movement state
        self.target_x = None
        self.target_y = None
        self.moving_to_target = False
        self.state = "idle"  # idle, turning, moving

        self.current_plan = []
        self.current_goal_index = 0
        self.plan_id = -1
        self.replan_requested = False
        self.executing = False

        # Subscriptions
        self.plan_sub = self.create_subscription(
            String, "/fleet/cuopt_plan", self.plan_callback, 10
        )
        
        # Subscribe to real odometry from Isaac Sim
        # Note: Isaac Sim publishes to /{robot_id}/chassis/odom
        self.odom_sub = self.create_subscription(
            Odometry, f"/{robot_id}/chassis/odom", self.odom_callback, 10
        )

        # Publishers
        self.status_pub = self.create_publisher(Int32, f"/{robot_id}/status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
        self.debug_pub = self.create_publisher(
            String, f"/{robot_id}/executor_debug", 10
        )
        self.robot_state_pub = self.create_publisher(
            String, f"/{robot_id}/robot_state", 10
        )

        # Timers
        self.move_timer = self.create_timer(0.1, self.move_control)
        self.state_timer = self.create_timer(1.0, self.publish_robot_state)

        # Nav2 Action Client (if use_nav2 is True)
        self.nav_action_client = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        
        if self.use_nav2:
            action_server_name = f"/{robot_id}/navigate_to_pose"
            self.nav_action_client = ActionClient(self, NavigateToPose, action_server_name)
            self.get_logger().info(f"Nav2 action client created: {action_server_name}")
            
            # Wait for Nav2 server to be available
            if self.nav_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info("Nav2 server available")
            else:
                self.get_logger().warn("Nav2 server not available, falling back to cmd_vel")
                self.use_nav2 = False

        self.get_logger().info(f"Task Executor initialized for {robot_id}")
        self.get_logger().info(f"Mode: {'Nav2' if self.use_nav2 else 'cmd_vel'}")
        self.get_logger().info(f"Subscribed to /{robot_id}/chassis/odom")
        self.get_logger().info(f"Publishing to /{robot_id}/cmd_vel")

    def odom_callback(self, msg):
        """Update position from real Isaac Sim odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract theta from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def publish_robot_state(self):
        """Publish robot state to /{robot_id}/robot_state for CuOpt."""
        current_task = -1
        target_waypoint = -1
        progress = 0.0
        
        if self.current_plan and self.current_goal_index < len(self.current_plan):
            current_task = self.current_plan[self.current_goal_index]
            target_waypoint = self.current_plan[-1]
            progress = self.current_goal_index / len(self.current_plan) if self.current_plan else 0.0
        
        state = {
            "robot_id": self.robot_id,
            "x": round(self.current_x, 3),
            "y": round(self.current_y, 3),
            "theta": round(self.current_theta, 3),
            "busy": self.executing,
            "current_task": current_task,
            "target_waypoint": target_waypoint,
            "progress": round(progress, 2),
            "plan_id": self.plan_id,
        }
        
        msg = String()
        msg.data = json.dumps(state)
        self.robot_state_pub.publish(msg)
        self.get_logger().debug(f"Published state: {state}")

    def navigate_to_pose(self, x: float, y: float, theta: float = 0.0) -> bool:
        """
        Navigate to pose using Nav2 action.
        
        Args:
            x: Target x position
            y: Target y position
            theta: Target orientation (radians)
            
        Returns:
            True if goal succeeded, False otherwise
        """
        if not self.use_nav2 or not self.nav_action_client:
            self.get_logger().warn("Nav2 not enabled, cannot navigate")
            return False
            
        # Check if server is available
        if not self.nav_action_client.server_is_ready():
            self.get_logger().error("Nav2 server not ready")
            return False
        
        # Create pose goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(theta / 2)
        goal_pose.pose.orientation.w = math.cos(theta / 2)
        
        # Create NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"Nav2: Sending goal ({x}, {y})")
        
        # Send goal
        self.nav_goal_handle = self.nav_action_client.send_goal_async(goal_msg)
        self.nav_goal_handle.add_done_callback(self.nav_goal_response_callback)
        
        # Wait for result (blocking)
        # For non-blocking, you'd handle this differently
        return True

    def nav_goal_response_callback(self, future):
        """Handle goal response from Nav2."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            return
            
        self.get_logger().info("Nav2 goal accepted")
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle goal result from Nav2."""
        try:
            result = future.result()
            if result.result == 2:  # SUCCESS
                self.get_logger().info("Nav2 goal succeeded")
            else:
                self.get_logger().warn(f"Nav2 goal finished with result: {result.result}")
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")

    def cancel_navigation(self):
        """Cancel current Nav2 navigation."""
        if self.nav_goal_handle:
            self.get_logger().info("Cancelling Nav2 goal")
            self.nav_goal_handle.cancel_goal()

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
            # Cancel Nav2 navigation if active
            if self.use_nav2:
                self.cancel_navigation()
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
            self.stop_robot()
            return

        self.executing = True
        self.publish_status(self.current_plan[self.current_goal_index])

        if self.current_goal_index < len(self.current_plan):
            task_id = self.current_plan[self.current_goal_index]
            target = WAREHOUSE_LOCATIONS[task_id]
            
            self.get_logger().info(
                f"EXECUTING: {self.robot_id} → Task {task_id} @ {target['name']}"
            )

            self.publish_debug(
                {
                    "state": "moving",
                    "task_id": task_id,
                    "location": target["name"],
                    "progress": f"{self.current_goal_index + 1}/{len(self.current_plan)}",
                    "plan_id": self.plan_id,
                }
            )

            # Start moving to target (non-blocking)
            self.start_move_to(target["x"], target["y"])
        else:
            self.get_logger().info(
                f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
            )
            self.executing = False
            self.publish_status(0)
            self.stop_robot()

    def start_move_to(self, target_x, target_y):
        """Start moving to target (non-blocking)."""
        self.target_x = target_x
        self.target_y = target_y
        self.moving_to_target = True
        
        if self.use_nav2:
            self.state = "nav2_navigating"
            # Use Nav2 for navigation
            result = self.navigate_to_pose(target_x, target_y, 0.0)
            if result:
                self.get_logger().info(f"Nav2 navigation started to ({target_x}, {target_y})")
            else:
                self.get_logger().error("Nav2 navigation failed to start")
        else:
            self.state = "turning"

    def move_control(self):
        """Timer-based movement control (10Hz)."""
        if not self.moving_to_target or self.target_x is None or self.target_y is None:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if arrived
        if dist < self.ARRIVAL_THRESHOLD:
            self.moving_to_target = False
            self.stop_robot()
            self.get_logger().info(f"ARRIVED at ({self.target_x}, {self.target_y})")
            
            # Move to next goal
            self.current_goal_index += 1
            
            # Handle replan during execution
            if self.replan_requested:
                self.get_logger().info("Replan received during execution, restarting...")
                self.replan_requested = False
                self.execute_plan()
                return
            
            # Continue with next goal or finish
            if self.current_goal_index < len(self.current_plan):
                task_id = self.current_plan[self.current_goal_index]
                target = WAREHOUSE_LOCATIONS[task_id]
                self.start_move_to(target["x"], target["y"])
            else:
                self.get_logger().info(
                    f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
                )
                self.executing = False
                self.publish_status(0)
                self.stop_robot()
            return

        # Calculate angle to target
        angle_to_target = math.atan2(dy, dx)
        
        # Calculate angle difference (handle wraparound)
        angle_diff = angle_to_target - self.current_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        twist = Twist()

        # State machine: turn first, then move
        if abs(angle_diff) > self.TURN_THRESHOLD:
            # Turning state
            self.state = "turning"
            twist.linear.x = 0.0
            twist.angular.z = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle_diff * 2.0))
            self.get_logger().debug(f"Turning: angle_diff={angle_diff:.3f}, angular={twist.angular.z:.3f}")
        else:
            # Moving state
            self.state = "moving"
            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = 0.0
            self.get_logger().debug(f"Moving: dist={dist:.3f}, heading={angle_to_target:.3f}")

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Publish zero velocity to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.state = "idle"

    def move_to(self, target_x, target_y):
        """Legacy method - now just sets target (non-blocking)."""
        self.start_move_to(target_x, target_y)

    def publish_status(self, task_id: int):
        msg = Int32()
        msg.data = task_id
        self.status_pub.publish(msg)

    def publish_debug(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)

    def shutdown_cleanup(self):
        """Stop robot on shutdown."""
        self.get_logger().info("Shutting down, stopping robot...")
        # Cancel Nav2 navigation if active
        if self.use_nav2:
            self.cancel_navigation()
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    
    # Create node to get parameters
    temp_node = rclpy.node.Node('temp')
    temp_node.declare_parameter('robot_id', 'amr1')
    temp_node.declare_parameter('use_nav2', False)
    
    robot_id = temp_node.get_parameter('robot_id').value
    use_nav2 = temp_node.get_parameter('use_nav2').value
    temp_node.destroy_node()
    
    node = TaskExecutor(robot_id=robot_id, use_nav2=use_nav2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
