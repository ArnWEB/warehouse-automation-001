#!/usr/bin/env python3
"""
Fleet Task Generator
- Generates warehouse tasks for the fleet
- Publishes to /fleet/tasks (Int32MultiArray)
- Can also trigger replanning via /fleet/plan_request
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import time
import random


WAREHOUSE_TASK_TYPES = [
    ('pickup_palletizer', 1),
    ('quality_check', 2),
    ('store_asrs', 3),
    ('retrieve_asrs', 4),
    ('move_to_staging', 6),
    ('move_to_outbound', 7),
]


class FleetTaskGenerator(Node):
    def __init__(self, interval: float = 5.0, max_tasks: int = 10):
        super().__init__('fleet_task_generator')
        
        self.interval = interval
        self.max_tasks = max_tasks
        self.task_count = 0
        
        self.task_pub = self.create_publisher(Int32MultiArray, '/fleet/tasks', 10)
        
        self.replan_pub = self.create_publisher(String, '/fleet/plan_request', 10)
        
        self.timer = self.create_timer(interval, self.generate_task)
        
        self.get_logger().info(f'Fleet Task Generator started - publishing every {interval}s')

    def generate_task(self):
        if self.task_count >= self.max_tasks:
            self.get_logger().info('Max tasks reached, stopping')
            return
        
        task_locations = [loc for _, loc in WAREHOUSE_TASK_TYPES]
        
        task = random.choice(task_locations)
        
        msg = Int32MultiArray()
        msg.data = [task]
        self.task_pub.publish(msg)
        
        self.task_count += 1
        
        self.get_logger().info(f'Generated task: location {task}')

    def trigger_replan(self, reason: str):
        import json
        msg = String()
        msg.data = json.dumps({'reason': reason})
        self.replan_pub.publish(msg)
        self.get_logger().info(f'Triggered replan: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = FleetTaskGenerator(interval=5.0, max_tasks=10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
