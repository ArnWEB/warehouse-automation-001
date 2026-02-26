#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(String, '/orders', 10)
        self.timer = self.create_timer(3.0, self.publish_order)
        self.order_count = 0
        
    def publish_order(self):
        self.order_count += 1
        
        # Create sample order
        order = {
            'order_id': f'ORDER-{self.order_count:03d}',
            'priority': (self.order_count % 5) + 1,
            'items': [f'ITEM-{i}' for i in range(1, (self.order_count % 3) + 2)],
            'quantities': [(self.order_count % 5) + 1] * len([f'ITEM-{i}' for i in range(1, (self.order_count % 3) + 2)]),
            'due_time': time.time() + 3600,  # 1 hour from now
            'status': 'pending',
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(order)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published order: {order["order_id"]} (priority: {order["priority"]})')

def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
