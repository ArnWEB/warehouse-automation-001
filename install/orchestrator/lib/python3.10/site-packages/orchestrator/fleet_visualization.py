#!/usr/bin/env python3
"""
Fleet Visualization Node
- Subscribes to fleet topics
- Publishes visualization data for rviz2
- Shows nodes (locations) and edges (routes)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
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


class FleetVisualization(Node):
    def __init__(self):
        super().__init__('fleet_visualization')
        
        self.plan_sub = self.create_subscription(
            String, '/fleet/cuopt_plan', self.plan_callback, 10)
        
        self.tasks_sub = self.create_subscription(
            String, '/fleet/tasks', self.tasks_callback, 10)
        
        self.viz_pub = self.create_publisher(String, '/fleet/viz', 10)
        
        self.current_plan = None
        self.current_tasks = []
        
        self.get_logger().info('Fleet Visualization initialized')

    def plan_callback(self, msg):
        try:
            self.current_plan = json.loads(msg.data)
            self.publish_viz()
        except Exception as e:
            self.get_logger().error(f'Failed to parse plan: {e}')

    def tasks_callback(self, msg):
        try:
            self.current_tasks = list(msg.data)
            self.publish_viz()
        except:
            pass

    def publish_viz(self):
        nodes = []
        edges = []
        
        for loc_id, loc in WAREHOUSE_LOCATIONS.items():
            node = {
                'id': loc_id,
                'name': loc['name'],
                'x': loc['x'],
                'y': loc['y'],
                'type': 'location'
            }
            
            if loc_id in self.current_tasks:
                node['type'] = 'task'
            
            if self.current_plan:
                for robot_id, assignment in self.current_plan.get('assignments', {}).items():
                    if loc_id in assignment.get('tasks', []):
                        node['assigned_to'] = robot_id
            
            nodes.append(node)
        
        for i in range(len(WAREHOUSE_LOCATIONS)):
            for j in range(len(WAREHOUSE_LOCATIONS)):
                if i != j:
                    edges.append({
                        'from': i,
                        'to': j,
                        'cost': self._distance(i, j)
                    })
        
        if self.current_plan:
            for robot_id, assignment in self.current_plan.get('assignments', {}).items():
                tasks = assignment.get('tasks', [])
                for k in range(len(tasks) - 1):
                    edges.append({
                        'from': tasks[k],
                        'to': tasks[k+1],
                        'type': 'route',
                        'robot': robot_id
                    })
        
        viz_data = {
            'nodes': nodes,
            'edges': edges,
            'robots': list(self.current_plan.get('assignments', {}).keys()) if self.current_plan else [],
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        msg = String()
        msg.data = json.dumps(viz_data)
        self.viz_pub.publish(msg)
        
        self.get_logger().info(f'Published viz: {len(nodes)} nodes, {len(edges)} edges')

    def _distance(self, i, j):
        loc_i = WAREHOUSE_LOCATIONS[i]
        loc_j = WAREHOUSE_LOCATIONS[j]
        return ((loc_j['x'] - loc_i['x'])**2 + (loc_j['y'] - loc_i['y'])**2) ** 0.5


def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
