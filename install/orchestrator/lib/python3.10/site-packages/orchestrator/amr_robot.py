#!/usr/bin/env python3
"""
AMR Robot Node - Simulates a robot executing tasks
Subscribes to /amrX/task and reports status
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random

class AMRRobot(Node):
    def __init__(self, robot_id):
        super().__init__(f'amr_robot_{robot_id}')
        self.robot_id = robot_id
        
        # Subscribe to tasks
        self.task_sub = self.create_subscription(
            String, f'/{robot_id}/task', self.task_callback, 10)
            
        # Publish status
        self.status_pub = self.create_publisher(String, f'/{robot_id}/status', 10)
        
        # Current state
        self.current_task = None
        self.battery = random.randint(70, 100)
        self.location = 'charging_station'
        self.state = 'idle'  # idle, moving, working, completed
        
        self.get_logger().info(f'{robot_id} started - Battery: {self.battery}%')
        
    def task_callback(self, msg):
        try:
            task = json.loads(msg.data)
            self.get_logger().info(
                f'{self.robot_id} received task: {task["task_id"]} '
                f'({task["task_type"]} at {task["location"]})')
            
            # Simulate task execution
            self.execute_task(task)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process task: {e}')
            
    def execute_task(self, task):
        """Simulate task execution"""
        self.current_task = task
        self.state = 'executing'
        
        # Simulate travel and work time
        execution_time = random.uniform(1.0, 3.0)
        
        self.get_logger().info(
            f'{self.robot_id} executing: {task["task_type"]} '
            f'at {task["location"]} ({execution_time:.1f}s)')
        
        # Update location
        self.location = task['location']
        
        # Publish status
        self.publish_status('executing', task)
        
    def publish_status(self, status, task=None):
        """Publish robot status"""
        status_msg = {
            'robot_id': self.robot_id,
            'state': status,
            'battery': self.battery,
            'location': self.location,
            'current_task': task['task_id'] if task else None,
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)


def main(robot_id='amr1'):
    rclpy.init(args=None)
    node = AMRRobot(robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'amr1'
    main(robot_id)
