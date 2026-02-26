#!/usr/bin/env python3
"""
CuOpt Client Node
- Subscribes to /fleet/tasks and /fleet/robot_states
- Calls NVIDIA cuOpt (or mock) for optimization
- Publishes optimized plans to /fleet/cuopt_plan

This node converts ROS2 → cuOpt API.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
import json
import time
import random
import numpy as np

try:
    import cuopt
    from cuopt import routing
    CUOPT_AVAILABLE = True
except ImportError:
    CUOPT_AVAILABLE = False
    print("cuOpt not available, using mock solver")


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


class CuOptClient(Node):
    def __init__(self, use_mock: bool = True):
        super().__init__('cuopt_client')
        
        self.use_mock = use_mock or not CUOPT_AVAILABLE
        
        if self.use_mock:
            self.get_logger().info('CuOpt Client: Using MOCK solver')
        else:
            self.get_logger().info('CuOpt Client: Using REAL cuOpt API')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.task_sub = self.create_subscription(
            Int32MultiArray, '/cuopt/trigger', self.tasks_callback, qos)
        
        self.robot_state_sub = self.create_subscription(
            String, '/fleet/robot_states', self.robot_states_callback, qos)
        
        self.plan_pub = self.create_publisher(String, '/fleet/cuopt_plan', qos)
        
        self.plan_pub_debug = self.create_publisher(String, '/fleet/cuopt_plan_debug', 10)
        
        self.pending_tasks = []
        self.robot_states = {}
        self.plan_count = 0
        
        self.get_logger().info('CuOpt Client Node initialized')

    def robot_states_callback(self, msg):
        try:
            states = json.loads(msg.data)
            self.robot_states = {r['robot_id']: r for r in states}
        except Exception as e:
            self.get_logger().warn(f'Failed to parse robot states: {e}')

    def tasks_callback(self, msg):
        task_ids = list(msg.data)
        if task_ids:
            self.pending_tasks.extend(task_ids)
            self.get_logger().info(f'Received {len(task_ids)} new tasks: {task_ids}')
            
            if len(self.pending_tasks) >= 1:
                if not self.robot_states:
                    self.robot_states = {
                        'amr1': {'robot_id': 'amr1', 'x': -5.0, 'y': 0.0},
                        'amr2': {'robot_id': 'amr2', 'x': -5.0, 'y': 2.0},
                        'amr3': {'robot_id': 'amr3', 'x': -5.0, 'y': -2.0},
                    }
                    self.get_logger().info('Using default robot positions')
                self.solve_and_publish()

    def solve_and_publish(self):
        if not self.robot_states:
            self.get_logger().warn('No robot states available, cannot optimize')
            return
            
        self.plan_count += 1
        
        if self.use_mock:
            plan = self._solve_mock()
        else:
            plan = self._solve_cuopt()
        
        msg = String()
        msg.data = json.dumps(plan)
        self.plan_pub.publish(msg)
        
        debug_msg = String()
        debug_msg.data = json.dumps({
            'plan_id': plan['plan_id'],
            'robot_assignments': {k: v['tasks'] for k, v in plan['assignments'].items()},
            'timestamp': time.time()
        })
        self.plan_pub_debug.publish(debug_msg)
        
        self.get_logger().info(
            f'Plan {plan["plan_id"]}: ' + 
            ', '.join([f"{k}→{v['tasks']}" for k, v in plan['assignments'].items()])
        )
        
        self.pending_tasks = []

    def _solve_mock(self):
        n_robots = len(self.robot_states)
        n_tasks = len(self.pending_tasks)
        
        assignments = {}
        tasks_per_robot = [[] for _ in range(n_robots)]
        
        for i, task_id in enumerate(self.pending_tasks):
            robot_idx = i % n_robots
            tasks_per_robot[robot_idx].append(task_id)
        
        for idx, (robot_id, state) in enumerate(self.robot_states.items()):
            assignments[robot_id] = {
                'tasks': tasks_per_robot[idx],
                'total_cost': random.uniform(10, 50),
                'estimated_time': len(tasks_per_robot[idx]) * 2.5
            }
        
        return {
            'plan_id': self.plan_count,
            'created_at': time.time(),
            'solver': 'mock' if self.use_mock else 'cuopt',
            'solve_time_ms': random.uniform(50, 200),
            'assignments': assignments,
            'total_cost': sum(a['total_cost'] for a in assignments.values()),
            'num_tasks': n_tasks
        }

    def _solve_cuopt(self):
        n_locations = len(WAREHOUSE_LOCATIONS)
        n_robots = len(self.robot_states)
        
        locations = list(WAREHOUSE_LOCATIONS.keys())
        cost_matrix = self._build_cost_matrix(locations)
        
        data_model = routing.DataModel(n_locations, n_robots)
        
        data_model.add_cost_matrix(cost_matrix)
        
        solver = routing.Solver(data_model)
        solution = solver.solve()
        
        assignments = {}
        routes = solution.get_routes()
        
        for robot_id, state in enumerate(self.robot_states.keys()):
            if robot_id < len(routes):
                route = routes[robot_id]
                task_ids = [t for t in route if t > 0]
                assignments[robot_id] = {
                    'tasks': task_ids,
                    'total_cost': solution.get_objective_value(),
                    'estimated_time': len(task_ids) * 2.5
                }
        
        return {
            'plan_id': self.plan_count,
            'created_at': time.time(),
            'solver': 'cuopt',
            'solve_time_ms': solution.get_solve_time(),
            'assignments': assignments,
            'total_cost': solution.get_objective_value(),
            'num_tasks': len(self.pending_tasks)
        }

    def _build_cost_matrix(self, locations):
        n = len(locations)
        cost = np.zeros((n, n))
        
        for i, loc_i in enumerate(locations):
            for j, loc_j in enumerate(locations):
                if i != j:
                    x1, y1 = WAREHOUSE_LOCATIONS[loc_i]['x'], WAREHOUSE_LOCATIONS[loc_i]['y']
                    x2, y2 = WAREHOUSE_LOCATIONS[loc_j]['x'], WAREHOUSE_LOCATIONS[loc_j]['y']
                    cost[i][j] = ((x2-x1)**2 + (y2-y1)**2) ** 0.5
        
        return cost


def main(args=None):
    rclpy.init(args=args)
    node = CuOptClient(use_mock=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
