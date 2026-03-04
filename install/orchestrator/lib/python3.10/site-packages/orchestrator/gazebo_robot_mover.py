#!/usr/bin/env python3
"""
Gazebo Robot Mover
- Subscribes to executor debug info
- Moves Gazebo models to target positions using set_model_state service
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
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


class GazeboRobotMover(Node):
    def __init__(self):
        super().__init__('gazebo_robot_mover')
        
        self.debug_subs = {}
        self.robot_targets = {}
        self.robot_positions = {}
        
        self.set_model_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        for robot_id in ['amr1', 'amr2', 'amr3']:
            sub = self.create_subscription(
                String,
                f'/{robot_id}/executor_debug',
                lambda msg, rid=robot_id: self.debug_callback(msg, rid),
                10)
            self.debug_subs[robot_id] = sub
            
            self.robot_targets[robot_id] = None
            self.robot_positions[robot_id] = {
                'x': -5.0 + (ord(robot_id[-1]) - ord('1')) * 2, 
                'y': 0.0
            }
        
        self.timer = self.create_timer(0.2, self.move_robots)
        
        self.get_logger().info('Gazebo Robot Mover initialized')

    def debug_callback(self, msg, robot_id):
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            
            if state == 'moving':
                task_id = data.get('task_id')
                if task_id is not None and task_id in WAREHOUSE_LOCATIONS:
                    target = WAREHOUSE_LOCATIONS[task_id]
                    self.robot_targets[robot_id] = {'x': target['x'], 'y': target['y']}
                    self.get_logger().info(f'{robot_id} target: {target["name"]}')
        except Exception as e:
            pass

    def move_robots(self):
        for robot_id, target in self.robot_targets.items():
            if target:
                current = self.robot_positions[robot_id]
                dx = target['x'] - current['x']
                dy = target['y'] - current['y']
                dist = math.sqrt(dx*dx + dy*dy)
                
                step = 0.3
                if dist > 0.1:
                    current['x'] += (dx / dist) * step
                    current['y'] += (dy / dist) * step
                else:
                    current['x'] = target['x']
                    current['y'] = target['y']
                
                self._set_gazebo_model(robot_id, current['x'], current['y'])

    def _set_gazebo_model(self, model_name, x, y):
        req = SetModelState.Request()
        
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.1
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        
        req.model_state = model_state
        
        try:
            future = self.set_model_client.call_async(req)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GazeboRobotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
