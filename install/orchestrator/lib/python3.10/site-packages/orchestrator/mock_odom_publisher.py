#!/usr/bin/env python3
"""
Mock Odometry Publisher for Testing
Simulates Isaac Sim odometry for testing task_executor without simulation

Usage:
    ros2 run orchestrator mock_odom_publisher --ros-args -p robot_id:=amr1 -p x:=17.98 -p y:=4.16
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


class MockOdometryPublisher(Node):
    def __init__(self, robot_id: str = "amr1", x: float = 17.98, y: float = 4.16):
        super().__init__(f"mock_odom_{robot_id}")
        
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.theta = 0.0
        
        self.odom_pub = self.create_publisher(
            Odometry, 
            f"/{robot_id}/chassis/odom", 
            10
        )
        
        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_odom)
        
        self.get_logger().info(f"Mock odometry publisher started for {robot_id}")
        self.get_logger().info(f"Position: ({x}, {y})")

    def publish_odom(self):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = self.robot_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from theta
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2)
        q.w = math.cos(self.theta / 2)
        odom.pose.pose.orientation = q
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    
    # Get parameters using rclpy
    node = rclpy.node.Node('temp')
    node.declare_parameter('robot_id', 'amr1')
    node.declare_parameter('x', 17.98)
    node.declare_parameter('y', 4.16)
    
    robot_id = node.get_parameter('robot_id').value
    x = node.get_parameter('x').value
    y = node.get_parameter('y').value
    node.destroy_node()
    
    # Create and spin the actual node
    mock_node = MockOdometryPublisher(robot_id, x, y)
    try:
        rclpy.spin(mock_node)
    except KeyboardInterrupt:
        pass
    finally:
        mock_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
