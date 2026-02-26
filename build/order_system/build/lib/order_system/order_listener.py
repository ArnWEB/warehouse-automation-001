#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        self.subscription = self.create_subscription(
            String, '/orders', self.order_callback, 10)
        
    def order_callback(self, msg):
        try:
            order = json.loads(msg.data)
            self.get_logger().info(
                f'Received order: {order["order_id"]} - '
                f'Items: {order["items"]}, Priority: {order["priority"]}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to parse order: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OrderListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
