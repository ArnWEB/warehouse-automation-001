#!/usr/bin/env python3
"""
Gazebo Fleet Controller
- Subscribes to odometry from executors
- Moves actual Gazebo models using set_model_state service
- Creates visual markers for the graph
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import json


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


class GazeboFleetController(Node):
    def __init__(self):
        super().__init__('gazebo_fleet_controller')
        
        self.odom_subs = {}
        self.target_positions = {}
        self.robot_positions = {}
        
        self.set_model_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        for robot_id in ['amr1', 'amr2', 'amr3']:
            sub = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.odom_subs[robot_id] = sub
            
            self.robot_positions[robot_id] = {'x': -5.0 + (ord(robot_id[-1]) - ord('1')) * 2, 'y': 0.0}
            self.target_positions[robot_id] = None
        
        self.viz_pub = self.create_publisher(String, '/visualization_marker_array', 10)
        
        self.timer = self.create_timer(0.1, self.move_robots)
        
        self.get_logger().info('Gazebo Fleet Controller initialized')

    def odom_callback(self, msg, robot_id):
        self.robot_positions[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def move_robots(self):
        for robot_id, target in self.target_positions.items():
            if target:
                self._move_robot(robot_id, target['x'], target['y'])
        
        self.publish_markers()

    def _move_robot(self, robot_id, x, y):
        req = SetModelState.Request()
        
        model_state = ModelState()
        model_state.model_name = robot_id
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.1
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        
        req.model_state = model_state
        
        future = self.set_model_client.call_async(req)
        
    def set_target(self, robot_id, x, y):
        self.target_positions[robot_id] = {'x': x, 'y': y}
        self.get_logger().info(f'Setting {robot_id} target: ({x}, {y})')

    def publish_markers(self):
        markers = []
        
        for loc_id, loc in WAREHOUSE_LOCATIONS.items():
            markers.append({
                'type': 'sphere',
                'ns': 'locations',
                'id': loc_id,
                'x': loc['x'],
                'y': loc['y'],
                'z': 0.5,
                'scale': 0.3,
                'color': [0.5, 0.5, 0.5, 0.5]
            })
        
        colors = [[0, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 1]]
        for i, (robot_id, pos) in enumerate(self.robot_positions.items()):
            markers.append({
                'type': 'cube',
                'ns': 'robots',
                'id': i,
                'x': pos['x'],
                'y': pos['y'],
                'z': 0.2,
                'scale': [0.6, 0.4, 0.2],
                'color': colors[i]
            })
        
        msg = String()
        msg.data = json.dumps(markers)
        self.viz_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFleetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
