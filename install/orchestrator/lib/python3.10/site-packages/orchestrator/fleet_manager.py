#!/usr/bin/env python3
"""
Fleet Manager Node
- Central brain for fleet orchestration
- Collects tasks and robot states
- Triggers cuOpt optimization
- Distributes plans to executors

Topics:
  /fleet/tasks (input) - New tasks to process
  /fleet/robot_states (input) - Robot position/status updates
  /fleet/cuopt_plan (output) - Optimized plans
  /fleet/plan_request (input) - Trigger replan
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, String
import json
import time


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.task_sub = self.create_subscription(
            Int32MultiArray, '/fleet/tasks', self.tasks_callback, 10)
        
        self.robot_state_sub = self.create_subscription(
            String, '/fleet/robot_states', self.robot_states_callback, qos)
        
        self.replan_request_sub = self.create_subscription(
            String, '/fleet/plan_request', self.replan_request_callback, 10)
        
        self.cuopt_trigger_pub = self.create_publisher(
            Int32MultiArray, '/cuopt/trigger', 10)
        
        self.status_pub = self.create_publisher(
            String, '/fleet/status', 10)
        
        self.plan_debug_pub = self.create_publisher(
            String, '/fleet/plan_debug', 10)
        
        self.pending_tasks = []
        self.robot_states = {}
        self.plan_id = 0
        
        self.get_logger().info('Fleet Manager initialized')

    def tasks_callback(self, msg):
        new_tasks = list(msg.data)
        if new_tasks:
            self.pending_tasks.extend(new_tasks)
            self.get_logger().info(f'Received {len(new_tasks)} tasks: {new_tasks}')
            self.trigger_optimization()

    def robot_states_callback(self, msg):
        try:
            states = json.loads(msg.data)
            self.robot_states = {r['robot_id']: r for r in states}
        except Exception as e:
            self.get_logger().warn(f'Failed to parse robot states: {e}')

    def replan_request_callback(self, msg):
        try:
            data = json.loads(msg.data)
            reason = data.get('reason', 'unknown')
            self.get_logger().info(f'Replan requested: {reason}')
            self.trigger_optimization()
        except:
            pass

    def trigger_optimization(self):
        if not self.pending_tasks or not self.robot_states:
            return
        
        self.plan_id += 1
        
        task_msg = Int32MultiArray()
        task_msg.data = self.pending_tasks
        self.cuopt_trigger_pub.publish(task_msg)
        
        status = {
            'plan_id': self.plan_id,
            'pending_tasks': self.pending_tasks,
            'num_robots': len(self.robot_states),
            'status': 'optimizing',
            'timestamp': time.time()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
        self.get_logger().info(
            f'Triggered cuOpt for plan {self.plan_id} with {len(self.pending_tasks)} tasks'
        )

    def publish_plan_debug(self, plan):
        msg = String()
        msg.data = json.dumps({
            'plan_id': plan['plan_id'],
            'assignments': {f'AMR{k+1}': v['tasks'] for k, v in plan.get('assignments', {}).items()},
            'total_cost': plan.get('total_cost', 0),
            'timestamp': time.time()
        })
        self.plan_debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
