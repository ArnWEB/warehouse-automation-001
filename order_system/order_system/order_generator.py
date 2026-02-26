#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class OrderGenerator(Node):
    def __init__(self):
        super().__init__('order_generator')
        self.publisher_ = self.create_publisher(String, '/orders', 10)
        self.order_count = 0
        
        # Create a timer - generates order every 5 seconds
        self.timer = self.create_timer(5.0, self.generate_order)
        
        self.get_logger().info('Order Generator Started - Publishing every 5 seconds')
        
    def generate_order(self):
        self.order_count += 1
        
        order = {
            'order_id': f'ORDER-{self.order_count:03d}',
            'priority': (self.order_count % 5) + 1,
            'items': [f'ITEM-{i}' for i in range(1, (self.order_count % 3) + 2)],
            'quantities': [(self.order_count % 5) + 1] * len([f'ITEM-{i}' for i in range(1, (self.order_count % 3) + 2)]),
            'due_time': time.time() + 3600,
            'status': 'pending',
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(order)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Generated: {order["order_id"]} (priority: {order["priority"]})')

def main(args=None):
    rclpy.init(args=args)
    node = OrderGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
