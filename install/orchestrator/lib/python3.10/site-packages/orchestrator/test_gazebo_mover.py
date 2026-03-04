#!/usr/bin/env python3
"""
Simple Gazebo Test Mover
- Moves amr1 in a circle to test Gazebo connection
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time


class SimpleGazeboMover(Node):
    def __init__(self):
        super().__init__('simple_gazebo_mover')
        
        self.get_logger().info('Waiting for Gazebo service...')
        self.set_model_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        if not self.set_model_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo service not available!')
        else:
            self.get_logger().info('Gazebo service connected!')
        
        self.angle = 0.0
        self.radius = 3.0
        self.center_x = 0.0
        self.center_y = 0.0
        
        self.timer = self.create_timer(0.1, self.move_robot)
        
        self.get_logger().info('Simple Gazebo Mover started - moving amr1 in circle')

    def move_robot(self):
        self.angle += 0.05
        
        x = self.center_x + self.radius * math.cos(self.angle)
        y = self.center_y + self.radius * math.sin(self.angle)
        
        self.get_logger().info(f'Moving amr1 to ({x:.2f}, {y:.2f})')
        
        req = SetModelState.Request()
        
        model_state = ModelState()
        model_state.model_name = 'amr1'
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.1
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0
        
        req.model_state = model_state
        
        future = self.set_model_client.call_async(req)
        
        if self.angle > 2 * math.pi:
            self.angle = 0


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGazeboMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
