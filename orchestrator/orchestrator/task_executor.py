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
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from typing import Optional
import json
import math


WAREHOUSE_LOCATIONS = {
    5: {"name": "waypoint_68", "x": 10.19, "y": 12.47, "orientation": 0.0},
    6: {"name": "waypoint_69", "x": 11.41, "y": 12.47, "orientation": 0.0},
    7: {"name": "waypoint_70", "x": 17.98, "y": 12.47, "orientation": 0.0},
    8: {"name": "waypoint_71", "x": 8.72, "y": 12.47, "orientation": 0.0},
    9: {"name": "waypoint_72", "x": 10.2, "y": 12.47, "orientation": 0.0},
    4: {"name": "waypoint_79", "x": 11.41, "y": 4.16, "orientation": 0.0},
    2: {"name": "waypoint_86", "x": 15.33, "y": 20.21, "orientation": 0.0},
    1: {"name": "waypoint_1", "x": 13.8859, "y": 19.81, "orientation": 90.0},
    0: {"name": "waypoint_0", "x": 15.15383, "y": 20.0, "orientation": 90.0},
    3: {"name": "waypoint_89", "x": 3.51, "y": 20.27, "orientation": 0.0},
}

DOCKING_STATIONS = {
    0: {"x": 15.15, "y": 17.03, "orientation": 90.0, "move_backward": True},
    1: {"x": 13.88591, "y": 15.9442, "orientation": 90.0, "move_backward": False},
}

PICKUP_WAYPOINTS = [0, 1]


class TaskExecutor(Node):
    # Movement parameters for iw_hub AMR
    LINEAR_SPEED = 1.0  # m/s (increased from 0.5)
    ANGULAR_SPEED = 2.0  # rad/s
    DOCKING_WAIT_TIME = 5.0  # seconds to wait at docking station
    ARRIVAL_THRESHOLD = 0.2  # meters (increased for Isaac Sim)
    TURN_THRESHOLD = 0.05  # radians (~3 degrees) - precise alignment

    def __init__(
        self,
        robot_id: str = "amr1",
        use_nav2: bool = False,
        use_global_namespace: bool = True,
    ):
        super().__init__(f"task_executor_{robot_id}")

        self.robot_id = robot_id
        self.use_nav2 = use_nav2
        self.use_global_namespace = use_global_namespace

        # Default positions matching cuopt_bridge ROBOTS config
        # amr1: charging_station (waypoint 80), amr2: staging (waypoint 75), amr3: asrs_output (waypoint 15)
        if robot_id == "amr1":
            self.current_x = 0.0
            self.current_y = 10.40
        elif robot_id == "amr2":
            self.current_x = 0.0
            self.current_y = 3.96
        elif robot_id == "amr3":
            self.current_x = 0.0
            self.current_y = 7.01
        else:
            self.current_x = 0.0
            self.current_y = 10.40
        self.current_theta = 0.0

        # Movement state
        self.target_x = None
        self.target_y = None
        self.target_orientation = None  # Target orientation in radians
        self.move_backward = False  # Use negative velocity for docking
        self.moving_to_target = False
        self.state = "idle"  # idle, turning, moving, aligning, to_dock, returning

        # Pickup sequence state
        self.pickup_mode = False
        self.pickup_stage = (
            None  # 'to_waypoint', 'waiting_at_dock', 'to_dock', 'returning'
        )
        self.docking_target = None  # dict with x, y
        self.original_waypoint = None  # dict with x, y, orientation, task_id
        self.dock_wait_timer = None  # Timer for waiting at dock
        self.original_use_nav2 = None  # Store original nav2 state for docking

        self.current_plan = []
        self.current_goal_index = 0
        self.plan_id = -1
        self.replan_requested = False
        self.executing = False
        self.force_cmd_vel = False

        # Subscriptions
        self.plan_sub = self.create_subscription(
            String, "/fleet/cuopt_plan", self.plan_callback, 10
        )

        # Subscribe to real odometry from Isaac Sim
        # Note: Isaac Sim publishes to /{robot_id}/chassis/odom (namespaced) OR /chassis/odom (global)
        # Determine if using global namespace or namespaced based on use_nav2 and use_global_namespace

        # Nav2 Action Client (if use_nav2 is True)
        self.nav_action_client = None
        self.nav_goal_future = None
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_goal_token = 0

        # Determine topic names based on namespace mode
        # use_global_namespace=True: global Nav2 (/navigate_to_pose, /chassis/odom, /cmd_vel)
        # use_global_namespace=False: namespaced Nav2 (/{robot_id}/navigate_to_pose, etc.)

        if self.use_nav2:
            if self.use_global_namespace:
                # Global namespace (single robot Nav2)
                self.odom_topic = "/chassis/odom"
                self.cmd_vel_topic = "/cmd_vel"
                action_server_name = "/navigate_to_pose"
            else:
                # Namespaced (multi-robot Nav2)
                self.odom_topic = f"/{robot_id}/chassis/odom"
                self.cmd_vel_topic = f"/{robot_id}/cmd_vel"
                action_server_name = f"/{robot_id}/navigate_to_pose"

            self.nav_action_client = ActionClient(
                self, NavigateToPose, action_server_name
            )
            self.get_logger().info(f"Nav2 action client created: {action_server_name}")

            # Wait for Nav2 server to be available
            if self.nav_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info("Nav2 server available")
            else:
                self.get_logger().warn(
                    "Nav2 server not available, falling back to cmd_vel"
                )
                self.use_nav2 = False
                self.odom_topic = f"/{robot_id}/chassis/odom"
                self.cmd_vel_topic = f"/{robot_id}/cmd_vel"
        else:
            # cmd_vel mode - always use namespaced topics
            self.odom_topic = f"/{robot_id}/chassis/odom"
            self.cmd_vel_topic = f"/{robot_id}/cmd_vel"

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # Publishers
        self.status_pub = self.create_publisher(Int32, f"/{robot_id}/status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(
            String, f"/{robot_id}/executor_debug", 10
        )
        self.robot_state_pub = self.create_publisher(
            String, f"/{robot_id}/robot_state", 10
        )

        # Timers
        self.move_timer = self.create_timer(0.1, self.move_control)
        self.state_timer = self.create_timer(1.0, self.publish_robot_state)

        self.get_logger().info(f"Task Executor initialized for {robot_id}")
        self.get_logger().info(
            f"Mode: {'Nav2 (global)' if self.use_nav2 and self.use_global_namespace else 'Nav2 (namespaced)' if self.use_nav2 else 'cmd_vel'}"
        )
        self.get_logger().info(f"Subscribed to {self.odom_topic}")
        self.get_logger().info(f"Publishing to {self.cmd_vel_topic}")

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
            progress = (
                self.current_goal_index / len(self.current_plan)
                if self.current_plan
                else 0.0
            )

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
        self.nav_goal_token += 1
        goal_token = self.nav_goal_token
        self.nav_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        self.nav_goal_future.add_done_callback(
            lambda future, token=goal_token: self.nav_goal_response_callback(
                future, token
            )
        )

        return True

    def nav_goal_response_callback(self, future, goal_token):
        """Handle goal response from Nav2."""
        # Ignore stale callbacks from older goals (e.g., after replans/cancels)
        if goal_token != self.nav_goal_token:
            self.get_logger().debug(
                f"Ignoring stale Nav2 goal response (token={goal_token})"
            )
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal response error: {e}")
            self.handle_nav2_goal_failure("goal_response_error")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            self.handle_nav2_goal_failure("rejected")
            return

        self.get_logger().info("Nav2 goal accepted")
        self.nav_goal_handle = goal_handle
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(
            lambda result_future, token=goal_token: self.nav_result_callback(
                result_future, token
            )
        )

    def nav_result_callback(self, future, goal_token):
        """Handle goal result from Nav2."""
        # Ignore stale callbacks from older goals (e.g., canceled due to replan)
        if goal_token != self.nav_goal_token:
            self.get_logger().debug(f"Ignoring stale Nav2 result (token={goal_token})")
            return

        try:
            result = future.result()
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Nav2 goal succeeded at ({self.target_x}, {self.target_y})"
                )
                self.moving_to_target = False
                self.advance_to_next_goal()
            elif status == GoalStatus.STATUS_CANCELED and self.replan_requested:
                self.get_logger().info("Nav2 goal canceled due to replan")
            else:
                self.get_logger().warn(f"Nav2 goal finished with status: {status}")
                self.handle_nav2_goal_failure(f"status_{status}")

        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            self.handle_nav2_goal_failure("result_exception")

    def advance_to_next_goal(self):
        """Advance plan index and execute next waypoint if available."""

        # Handle pickup sequence (keep using Nav2 for all movements)
        if self.pickup_mode:
            if self.pickup_stage == "to_waypoint":
                # Using Nav2 for docking - keep nav2 enabled
                self.get_logger().info("Moving to docking station (using Nav2)...")
                self.pickup_stage = "to_dock"
                dock_orientation = math.radians(
                    self.docking_target.get("orientation", 0.0)
                )
                # Use cmd_vel for docking
                self.start_move_to(
                    self.docking_target["x"],
                    self.docking_target["y"],
                    dock_orientation,
                    force_cmd_vel=True,
                )
                return
            elif self.pickup_stage == "to_dock":
                self.get_logger().info(
                    f"Arrived at dock! Waiting {self.DOCKING_WAIT_TIME} seconds..."
                )
                self.pickup_stage = "waiting_at_dock"
                self.dock_wait_timer = self.create_timer(
                    self.DOCKING_WAIT_TIME, self.dock_wait_callback
                )
                return
            elif self.pickup_stage == "returning":
                self.get_logger().info(
                    "Returned to waypoint, continuing to next task..."
                )
                self.pickup_mode = False
                self.pickup_stage = None
                self.force_cmd_vel = False
                # Continue using Nav2
                self.current_goal_index += 1
            else:
                # Unknown stage, reset
                self.pickup_mode = False
                self.pickup_stage = None
                self.force_cmd_vel = False
                self.current_goal_index += 1
        else:
            # Normal case: increment index
            self.current_goal_index += 1

        # Handle replan during execution
        if self.replan_requested:
            self.get_logger().info("Replan received during execution, restarting...")
            self.replan_requested = False
            self.execute_plan()
            return

        # Check if the task we just completed was a pickup task (initiate docking AFTER arrival)
        completed_task_id = self.current_plan[self.current_goal_index - 1]
        if completed_task_id in PICKUP_WAYPOINTS:
            target = WAREHOUSE_LOCATIONS[completed_task_id]
            self.pickup_mode = True
            self.pickup_stage = "to_waypoint"  # We are at waypoint, next is dock
            self.original_waypoint = {
                "x": target["x"],
                "y": target["y"],
                "orientation": target.get("orientation", 0.0),
                "task_id": completed_task_id,
            }
            self.docking_target = DOCKING_STATIONS.get(completed_task_id)
            self.docking_move_backward = self.docking_target.get("move_backward", False)
            self.get_logger().info(
                f"PICKUP TASK {completed_task_id} complete! Initiating dock sequence -> {self.docking_target}"
            )
            self.get_logger().info(
                f"DOCKING: approach_move_backward={self.docking_move_backward}, will return with backward={not self.docking_move_backward}"
            )

            # Use Nav2 for docking (will return with cmd_vel)
            dock_orientation = math.radians(self.docking_target.get("orientation", 0.0))
            self.start_move_to(
                self.docking_target["x"],
                self.docking_target["y"],
                dock_orientation,
                force_cmd_vel=False,  # Use Nav2 for docking approach
            )
            return

        # Continue with next goal or finish
        if self.current_goal_index < len(self.current_plan):
            task_id = self.current_plan[self.current_goal_index]
            target = WAREHOUSE_LOCATIONS[task_id]

            self.get_logger().info(
                f"Moving to next goal: Task {task_id} @ {target['name']}"
            )
            orientation_deg = target.get("orientation", 0.0)
            orientation_rad = math.radians(orientation_deg)
            self.start_move_to(target["x"], target["y"], orientation_rad)
        else:
            self.get_logger().info(
                f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
            )
            self.executing = False
            self.publish_status(0)
            self.stop_robot()
            self.moving_to_target = False

    def handle_nav2_goal_failure(self, reason: str):
        """Handle failed Nav2 goals safely to avoid undefined behavior."""
        self.get_logger().error(f"Nav2 goal failed ({reason}); stopping execution")
        self.moving_to_target = False
        self.executing = False
        self.publish_status(0)
        self.stop_robot()

    def cancel_navigation(self):
        """Cancel current Nav2 navigation."""
        if self.nav_goal_handle:
            self.get_logger().info("Cancelling Nav2 goal")
            self.nav_goal_handle.cancel_goal_async()

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            assignments = plan.get("assignments", {})

            # Use robot_id directly as key (e.g., "amr1", "amr2")
            if self.robot_id in assignments:
                new_tasks = assignments[self.robot_id]["route"]
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
                    "pickup_mode": self.pickup_mode,
                }
            )

            # Start moving to target (non-blocking) with orientation
            orientation_deg = target.get("orientation", 0.0)
            orientation_rad = math.radians(orientation_deg)
            self.start_move_to(target["x"], target["y"], orientation_rad)
        else:
            self.get_logger().info(
                f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
            )
            self.executing = False
            self.publish_status(0)
            self.stop_robot()

    def start_move_to(
        self,
        target_x: float,
        target_y: float,
        orientation: Optional[float] = None,
        backward: bool = False,
        force_cmd_vel: bool = False,
    ):
        """Start moving to target (non-blocking)."""
        self.target_x = target_x
        self.target_y = target_y
        self.target_orientation = orientation
        self.move_backward = backward
        self.moving_to_target = True
        self.force_cmd_vel = force_cmd_vel

        if self.use_nav2 and not force_cmd_vel:
            self.state = "nav2_navigating"
            # Use Nav2 for navigation
            result = self.navigate_to_pose(
                target_x, target_y, orientation if orientation is not None else 0.0
            )
            if result:
                self.get_logger().info(
                    f"Nav2 navigation started to ({target_x}, {target_y}, orientation={orientation})"
                )
            else:
                self.get_logger().error("Nav2 navigation failed to start")
        else:
            # Cancel any active Nav2 navigation before switching to cmd_vel
            if self.nav_goal_handle:
                self.get_logger().info("Canceling Nav2 goal to switch to cmd_vel")
                self.nav_goal_handle.cancel_goal_async()
                self.nav_goal_handle = None
            self.state = "turning"

    def move_control(self):
        """Timer-based movement control (10Hz)."""
        # Skip if using Nav2 (Nav2 handles movement) unless force_cmd_vel is set
        if self.use_nav2 and not self.force_cmd_vel:
            return

        if not self.moving_to_target or self.target_x is None or self.target_y is None:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if arrived at target position
        if dist < self.ARRIVAL_THRESHOLD:
            self.get_logger().info(
                f"ARRIVED! dist={dist:.3f}, target_orientation={self.target_orientation}"
            )

            # Check if we need to align to target orientation
            if self.target_orientation is not None:
                self.get_logger().info(
                    f"ALIGNING: current_theta={math.degrees(self.current_theta):.1f} deg, target={math.degrees(self.target_orientation):.1f} deg"
                )
                self.state = "aligning"
                # Align to target orientation
                angle_diff = self.target_orientation - self.current_theta
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if abs(angle_diff) > self.TURN_THRESHOLD:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = max(
                        -self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle_diff * 2.0)
                    )
                    self.cmd_vel_pub.publish(twist)
                    return
                else:
                    # Alignment complete
                    self.get_logger().info("Orientation aligned!")
                    self.target_orientation = None
                    self.moving_to_target = False
            else:
                self.moving_to_target = False

            # Position reached and aligned (or no alignment needed)
            self.stop_robot()
            self.get_logger().info(
                f"ARRIVED & ALIGNED at ({self.target_x}, {self.target_y})"
            )

            # Handle pickup sequence if active
            if self.pickup_mode:
                if self.pickup_stage == "to_waypoint":
                    self.get_logger().info("Moving to docking station via Nav2...")
                    self.pickup_stage = "to_dock"
                    self.docking_move_backward = self.docking_target.get(
                        "move_backward", False
                    )
                    self.get_logger().info(
                        f"DOCKING: approach_move_backward={self.docking_move_backward}, will return with backward={not self.docking_move_backward}"
                    )
                    dock_orientation = math.radians(
                        self.docking_target.get("orientation", 0.0)
                    )
                    # Using Nav2 for docking approach
                    self.start_move_to(
                        self.docking_target["x"],
                        self.docking_target["y"],
                        dock_orientation,
                        force_cmd_vel=False,  # Use Nav2 for docking approach
                    )
                    return
                elif self.pickup_stage == "to_dock":
                    self.get_logger().info(
                        f"Arrived at dock! Waiting {self.DOCKING_WAIT_TIME} seconds..."
                    )
                    self.pickup_stage = "waiting_at_dock"
                    self.dock_wait_timer = self.create_timer(
                        self.DOCKING_WAIT_TIME, self.dock_wait_callback
                    )
                    return
                elif self.pickup_stage == "returning":
                    self.get_logger().info(
                        "Returned to waypoint, continuing to next task..."
                    )
                    self.pickup_mode = False
                    self.pickup_stage = None
                    self.force_cmd_vel = False
                    self.docking_move_backward = False

                    # Continue using Nav2
                    self.current_goal_index += 1
                return

            # Check if current task is a pickup task (initiate pickup sequence AFTER arrival)
            current_task_id = self.current_plan[self.current_goal_index]
            if current_task_id in PICKUP_WAYPOINTS:
                target = WAREHOUSE_LOCATIONS[current_task_id]
                self.pickup_mode = True
                self.pickup_stage = "to_waypoint"  # We are at waypoint, next is dock
                self.original_waypoint = {
                    "x": target["x"],
                    "y": target["y"],
                    "orientation": target.get("orientation", 0.0),
                    "task_id": current_task_id,
                }
                self.docking_target = DOCKING_STATIONS.get(current_task_id)
                self.docking_move_backward = self.docking_target.get(
                    "move_backward", False
                )
                self.get_logger().info(
                    f"PICKUP TASK {current_task_id} complete! Initiating dock sequence -> {self.docking_target}"
                )
                self.get_logger().info(
                    f"DOCKING: approach_move_backward={self.docking_move_backward}, will return with backward={not self.docking_move_backward}"
                )
                # Start moving to docking station using Nav2
                dock_orientation = math.radians(
                    self.docking_target.get("orientation", 0.0)
                )
                self.start_move_to(
                    self.docking_target["x"],
                    self.docking_target["y"],
                    dock_orientation,
                    force_cmd_vel=False,  # Use Nav2 for docking approach
                )
                return

            # Move to next goal
            self.current_goal_index += 1

            # Handle replan during execution
            if self.replan_requested:
                self.get_logger().info(
                    "Replan received during execution, restarting..."
                )
                self.replan_requested = False
                self.execute_plan()
                return

            # Continue with next goal or finish
            if self.current_goal_index < len(self.current_plan):
                task_id = self.current_plan[self.current_goal_index]
                target = WAREHOUSE_LOCATIONS[task_id]

                # Check if next task is a pickup
                if task_id in PICKUP_WAYPOINTS:
                    self.pickup_mode = True
                    self.pickup_stage = "to_waypoint"
                    self.original_waypoint = {
                        "x": target["x"],
                        "y": target["y"],
                        "orientation": target.get("orientation", 0.0),
                        "task_id": task_id,
                    }
                    self.docking_target = DOCKING_STATIONS.get(task_id)

                orientation_deg = target.get("orientation", 0.0)
                orientation_rad = math.radians(orientation_deg)
                self.start_move_to(target["x"], target["y"], orientation_rad)
            else:
                self.get_logger().info(
                    f"PLAN COMPLETE: {self.robot_id} finished all tasks!"
                )
                self.executing = False
                self.force_cmd_vel = False
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
            twist.angular.z = max(
                -self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, angle_diff * 2.0)
            )
            self.get_logger().debug(
                f"Turning: angle_diff={angle_diff:.3f}, angular={twist.angular.z:.3f}"
            )
        else:
            # Moving state
            self.state = "moving"
            if self.move_backward:
                twist.linear.x = -self.LINEAR_SPEED  # Go backwards
                self.get_logger().debug(f"Moving BACKWARD: dist={dist:.3f}")
            else:
                twist.linear.x = self.LINEAR_SPEED  # Go forward
                self.get_logger().debug(f"Moving FORWARD: dist={dist:.3f}")
            twist.angular.z = 0.0

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
        # Cancel dock wait timer
        if self.dock_wait_timer is not None:
            self.dock_wait_timer.cancel()

    def dock_wait_callback(self):
        """Callback fired after waiting at docking station."""
        self.get_logger().info("Dock wait complete! Returning to waypoint...")
        self.pickup_stage = "returning"
        # Cancel the timer
        if self.dock_wait_timer is not None:
            self.dock_wait_timer.cancel()
            self.dock_wait_timer = None

        # Return to original waypoint using cmd_vel with opposite direction of approach
        return_backward = not self.docking_move_backward
        self.get_logger().info(
            f"RETURNING: approach_backward={self.docking_move_backward}, returning_backward={return_backward}"
        )
        self.start_move_to(
            self.original_waypoint["x"],
            self.original_waypoint["y"],
            self.original_waypoint["orientation"],
            backward=return_backward,  # Opposite of approach direction
            force_cmd_vel=True,
        )


def main(args=None):
    rclpy.init(args=args)

    # Create node to get parameters
    temp_node = rclpy.node.Node("temp")
    temp_node.declare_parameter("robot_id", "amr1")
    temp_node.declare_parameter("use_nav2", False)
    temp_node.declare_parameter("use_global_namespace", True)

    robot_id = temp_node.get_parameter("robot_id").value
    use_nav2 = temp_node.get_parameter("use_nav2").value
    use_global_namespace = temp_node.get_parameter("use_global_namespace").value
    temp_node.destroy_node()

    node = TaskExecutor(
        robot_id=robot_id, use_nav2=use_nav2, use_global_namespace=use_global_namespace
    )
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
