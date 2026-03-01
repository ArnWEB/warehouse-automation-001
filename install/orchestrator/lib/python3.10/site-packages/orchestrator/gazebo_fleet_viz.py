#!/usr/bin/env python3
"""
Gazebo Fleet Visualization Node
- Subscribes to /fleet/viz for graph structure
- Creates visual markers in Gazebo for nodes and edges
- Moves AMR robots to their target positions
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState, GetModelList
from gazebo_msgs.msg import ModelState
import json
import time


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


class GazeboFleetViz(Node):
    def __init__(self):
        super().__init__('gazebo_fleet_viz')
        
        self.viz_sub = self.create_subscription(
            String, '/fleet/viz', self.viz_callback, 10)
        
        self.odom_subs = {}
        self.robot_positions = {
            'amr1': {'x': -5.0, 'y': 0.0},
            'amr2': {'x': -5.0, 'y': 2.0},
            'amr3': {'x': -5.0, 'y': -2.0},
        }
        
        for robot_id in ['amr1', 'amr2', 'amr3']:
            sub = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.odom_subs[robot_id] = sub
        
        self.marker_pub = self.create_publisher(String, '/visualization_marker', 10)
        
        self.current_viz = None
        
        self.timer = self.create_timer(0.5, self.update_gazebo)
        
        self.get_logger().info('Gazebo Fleet Viz initialized')

    def odom_callback(self, msg, robot_id):
        self.robot_positions[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def viz_callback(self, msg):
        try:
            self.current_viz = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse viz: {e}')

    def update_gazebo(self):
        if not self.current_viz:
            return
        
        nodes = self.current_viz.get('nodes', [])
        edges = self.current_viz.get('edges', [])
        robots = self.current_viz.get('robots', [])
        
        markers = []
        
        for node in nodes:
            marker = {
                'type': 'sphere',
                'id': node['id'],
                'x': node['x'],
                'y': node['y'],
                'z': 0.5,
                'scale_x': 0.3,
                'scale_y': 0.3,
                'scale_z': 0.3,
                'color': self._get_node_color(node),
            }
            markers.append(marker)
        
        route_edges = [e for e in edges if e.get('type') == 'route']
        for i, edge in enumerate(route_edges):
            from_node = WAREHOUSE_LOCATIONS.get(edge['from'], {'x': 0, 'y': 0})
            to_node = WAREHOUSE_LOCATIONS.get(edge['to'], {'x': 0, 'y': 0})
            
            marker = {
                'type': 'line',
                'id': 100 + i,
                'points': [
                    {'x': from_node['x'], 'y': from_node['y'], 'z': 0.3},
                    {'x': to_node['x'], 'y': to_node['y'], 'z': 0.3}
                ],
                'color': {'r': 1.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
            }
            markers.append(marker)
        
        for robot_id, pos in self.robot_positions.items():
            idx = {'amr1': 0, 'amr2': 1, 'amr3': 2}.get(robot_id, 0)
            color = self._get_robot_color(idx)
            
            marker = {
                'type': 'cube',
                'id': 200 + idx,
                'x': pos['x'],
                'y': pos['y'],
                'z': 0.2,
                'scale_x': 0.6,
                'scale_y': 0.4,
                'scale_z': 0.2,
                'color': color,
                'label': robot_id
            }
            markers.append(marker)
        
        viz_msg = String()
        viz_msg.data = json.dumps(markers)
        self.marker_pub.publish(viz_msg)

    def _get_node_color(self, node):
        if node.get('assigned_to'):
            return {'r': 1.0, 'g': 0.5, 'b': 0.0, 'a': 1.0}
        elif node.get('type') == 'task':
            return {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
        else:
            return {'r': 0.5, 'g': 0.5, 'b': 0.5, 'a': 0.5}

    def _get_robot_color(self, idx):
        colors = [
            {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0},
            {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0},
            {'r': 1.0, 'g': 0.0, 'b': 1.0, 'a': 1.0},
        ]
        return colors[idx % len(colors)]


def main(args=None):
    rclpy.init(args=args)
    node = GazeboFleetViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
