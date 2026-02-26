#!/usr/bin/env python3
"""
Orchestrator - Executes CuOpt plans
- Subscribes to /cuopt/plan
- Assigns tasks to robots
- Publishes robot commands
- Monitors execution
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')
        
        # Subscribe to CuOpt plans
        self.plan_sub = self.create_subscription(
            String, '/cuopt/plan', self.plan_callback, 10)
        
        # Robot task publishers
        self.amr1_task_pub = self.create_publisher(String, '/amr1/task', 10)
        self.amr2_task_pub = self.create_publisher(String, '/amr2/task', 10)
        self.amr3_task_pub = self.create_publisher(String, '/amr3/task', 10)
        
        # Fleet control publisher
        self.fleet_control_pub = self.create_publisher(String, '/fleet/control', 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/orchestrator/status', 10)
        
        # Track active tasks
        self.active_tasks = {}
        self.completed_tasks = {}
        self.task_count = 0
        
        self.get_logger().info('Orchestrator Started')
        self.get_logger().info('Ready to execute CuOpt plans')
        
    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f'Received plan: {plan["plan_id"]} for {plan["order_id"]}')
            
            # Execute the plan
            self.execute_plan(plan)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process plan: {e}')
            
    def execute_plan(self, plan):
        """Execute plan by sending tasks to robots"""
        robot_id = plan['assigned_robot']['robot_id']
        tasks = plan['tasks']
        
        self.get_logger().info(
            f'Executing plan: {plan["plan_id"]} on {robot_id} '
            f'({len(tasks)} tasks)')
        
        # Send tasks to the assigned robot
        for task in tasks:
            self.task_count += 1
            task['plan_id'] = plan['plan_id']
            task['order_id'] = plan['order_id']
            task['assigned_at'] = time.time()
            task['status'] = 'assigned'
            
            # Store task
            self.active_tasks[task['task_id']] = task
            
            # Send to robot
            self.send_task_to_robot(robot_id, task)
            
        # Publish execution status
        self.publish_status(plan, 'executing')
        
    def send_task_to_robot(self, robot_id, task):
        """Send task to specific robot"""
        # Select publisher based on robot ID
        if robot_id == 'amr1':
            pub = self.amr1_task_pub
        elif robot_id == 'amr2':
            pub = self.amr2_task_pub
        elif robot_id == 'amr3':
            pub = self.amr3_task_pub
        else:
            self.get_logger().error(f'Unknown robot: {robot_id}')
            return
            
        # Create task message
        task_msg = {
            'task_id': task['task_id'],
            'task_type': task['task_type'],
            'location': task['location'],
            'zone': task['zone'],
            'coords': task['coords'],
            'sequence': task['sequence'],
            'plan_id': task['plan_id'],
            'order_id': task['order_id'],
            'status': 'assigned',
            'assigned_at': task['assigned_at']
        }
        
        msg = String()
        msg.data = json.dumps(task_msg)
        pub.publish(msg)
        
        self.get_logger().info(
            f'Sent task {task["task_id"]} to {robot_id}: '
            f'{task["task_type"]} at {task["location"]}')
            
    def publish_status(self, plan, status):
        """Publish orchestrator status"""
        status_msg = {
            'orchestrator_status': status,
            'plan_id': plan['plan_id'],
            'order_id': plan['order_id'],
            'robot_id': plan['assigned_robot']['robot_id'],
            'num_tasks': len(plan['tasks']),
            'active_tasks': len(self.active_tasks),
            'completed_tasks': len(self.completed_tasks),
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
