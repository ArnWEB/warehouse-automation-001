#!/usr/bin/env python3
"""
Robot State Monitor Node
- Monitors all AMR robot states
- Publishes robot positions and status

Subscribes to:
  /amr{1,2,3}/odom - Odometry data
  /amr{1,2,3}/status - Robot status (busy/idle)

Publishes:
  /fleet/robot_states - Combined robot states
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import json
import time


WAREHOUSE_LOCATIONS = {
    0: {'name': 'charging_station', 'x': -5.0, 'y': 0.0},
    1: {'name': 'palletizer', 'x': -8.0, 'y': 3.0},
    2: {'name': 'quality_check', 'x': -4.0, 'y': 3.0},
    3: {'name': 'asrs_input', 'x': 8.0, 'y': 3.0},
    4: {'name': 'asrs_storage', 'x': 8.0, 'y': 5.0},
    5: {'name': 'asrs_output', 'x': 8.0, 'y': 7.0},
    6: {'name': 'staging', 'x': 0.0, 'y': -5.0},
    7: {'name': 'outbound', 'x': 5.0, 'y': -5.0},
}


class RobotStateMonitor(Node):
    def __init__(self, num_robots: int = 3):
        super().__init__('robot_state_monitor')
        
        self.num_robots = num_robots
        self.robot_states = {}
        
        self.odom_subs = []
        self.status_subs = []
        
        for i in range(1, num_robots + 1):
            robot_id = f'amr{i}'
            
            odom_sub = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.odom_subs.append(odom_sub)
            
            status_sub = self.create_subscription(
                Int32,
                f'/{robot_id}/status',
                lambda msg, rid=robot_id: self.status_callback(msg, rid),
                10)
            self.status_subs.append(status_sub)
            
            self.robot_states[robot_id] = {
                'robot_id': robot_id,
                'x': -5.0,
                'y': 0.0,
                'theta': 0.0,
                'busy': False,
                'current_task': -1,
                'battery_level': 85.0,
                'last_update': time.time()
            }
        
        self.fleet_pub = self.create_publisher(String, '/fleet/robot_states', 10)
        
        self.timer = self.create_timer(1.0, self.publish_states)
        
        self.get_logger().info(f'Robot State Monitor initialized with {num_robots} robots')

    def odom_callback(self, msg: Odometry, robot_id: str):
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        import math
        theta = 2 * math.atan2(orient.z, orient.w)
        
        self.robot_states[robot_id]['x'] = pos.x
        self.robot_states[robot_id]['y'] = pos.y
        self.robot_states[robot_id]['theta'] = theta
        self.robot_states[robot_id]['last_update'] = time.time()

    def status_callback(self, msg: Int32, robot_id: str):
        status = msg.data
        self.robot_states[robot_id]['busy'] = status > 0
        self.robot_states[robot_id]['current_task'] = status if status > 0 else -1

    def publish_states(self):
        states_list = list(self.robot_states.values())
        
        msg = String()
        msg.data = json.dumps(states_list)
        self.fleet_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMonitor(num_robots=3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
