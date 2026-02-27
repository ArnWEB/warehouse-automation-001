#!/usr/bin/env python3
"""
Gazebo Robot Mover using Velocity Commands
- Uses /cmd_vel topic to move robots
- Subscribes to plan and publishes velocity commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import math


WAREHOUSE_LOCATIONS = {
    0: {'name': 'charging', 'x': -5.0, 'y': 0.0},
    1: {'name': 'palletizer', 'x': -8.0, 'y': 3.0},
    2: {'name': 'quality_check', 'x': -4.0, 'y': 3.0},
    3: {'name': 'asrs_in', 'x': 8.0, 'y': 3.0},
    4: {'name': 'asrs_store', 'x': 8.0, 'y': 5.0},
    5: {'name': 'asrs_out', 'x': 8.0, 'y': 7.0},
    6: {'name': 'staging', 'x': 0.0, 'y': -5.0},
    7: {'name': 'outbound', 'x': 5.0, 'y': -5.0},
}

ROBOT_START_POSITIONS = {
    'amr1': {'x': -5.0, 'y': 0.0, 'theta': 0.0},
    'amr2': {'x': -5.0, 'y': 2.0, 'theta': 0.0},
    'amr3': {'x': -5.0, 'y': -2.0, 'theta': 0.0},
}


class GazeboVelocityMover(Node):
    def __init__(self):
        super().__init__('gazebo_velocity_mover')
        
        self.plan_sub = self.create_subscription(
            String, '/fleet/cuopt_plan', self.plan_callback, 10)
        
        self.cmd_vel_pubs = {}
        for robot_id in ['amr1', 'amr2', 'amr3']:
            self.cmd_vel_pubs[robot_id] = self.create_publisher(
                Twist, f'/{robot_id}/cmd_vel', 10)
        
        self.robot_positions = {k: dict(v) for k, v in ROBOT_START_POSITIONS.items()}
        self.robot_targets = {k: None for k in ROBOT_START_POSITIONS}
        
        self.timer = self.create_timer(0.1, self.move_robots)
        
        self.get_logger().info('Gazebo Velocity Mover initialized')

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            assignments = plan.get('assignments', {})
            
            for robot_id, assignment in assignments.items():
                tasks = assignment.get('tasks', [])
                if tasks:
                    first_task = tasks[0]
                    if first_task in WAREHOUSE_LOCATIONS:
                        target = WAREHOUSE_LOCATIONS[first_task]
                        self.robot_targets[robot_id] = {'x': target['x'], 'y': target['y']}
                        self.get_logger().info(f'{robot_id} assigned to: {target["name"]}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse plan: {e}')

    def move_robots(self):
        for robot_id in self.cmd_vel_pubs.keys():
            target = self.robot_targets.get(robot_id)
            if target:
                current = self.robot_positions[robot_id]
                
                dx = target['x'] - current['x']
                dy = target['y'] - current['y']
                dist = math.sqrt(dx*dx + dy*dy)
                
                twist = Twist()
                
                if dist > 0.2:
                    angle_to_target = math.atan2(dy, dx)
                    angle_diff = angle_to_target - current['theta']
                    
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    twist.linear.x = 0.5
                    twist.angular.z = angle_diff * 2.0
                    
                    current['x'] += math.cos(current['theta']) * 0.05
                    current['y'] += math.sin(current['theta']) * 0.05
                    current['theta'] += twist.angular.z * 0.1
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                self.cmd_vel_pubs[robot_id].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboVelocityMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
