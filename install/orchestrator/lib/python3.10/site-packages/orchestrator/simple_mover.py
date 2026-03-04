#!/usr/bin/env python3
"""
Simple Gazebo Mover - Uses set_model_state service properly
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time


class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        
        self.get_logger().info('Waiting for Gazebo service...')
        self.client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        self.client.wait_for_service()
        self.get_logger().info('Service available!')
        
        self.timer = self.create_timer(0.2, self.move)
        self.angle = 0.0
        
    def move(self):
        self.angle += 0.1
        
        x = -5.0 + 3.0 * math.cos(self.angle)
        y = 3.0 * math.sin(self.angle)
        
        self.get_logger().info(f'Moving amr1 to ({x:.2f}, {y:.2f})')
        
        req = SetModelState.Request()
        req.model_state.model_name = 'amr1'
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = 0.1
        req.model_state.pose.orientation.x = 0.0
        req.model_state.pose.orientation.y = 0.0
        req.model_state.pose.orientation.z = 0.0
        req.model_state.pose.orientation.w = 1.0
        
        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
