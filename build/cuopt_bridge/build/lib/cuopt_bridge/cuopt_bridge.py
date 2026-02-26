#!/usr/bin/env python3
"""
CuOpt Mock Bridge (NVIDIA CuOpt Simulation)
- Subscribes to /orders
- Creates optimized task plan for robot fleet
- Publishes plan to /cuopt/plan

Real CuOpt features simulated:
- Task dependencies
- Time windows
- Robot capabilities
- Cost optimization
- Multi-objective routing
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random

# Warehouse locations (x, y coordinates in meters)
LOCATIONS = {
    'inbound_dock':      {'x': 0.0,  'y': 0.0,  'zone': 'Z1'},
    'palletizer':        {'x': 5.0,  'y': 0.0,  'zone': 'Z1'},
    'quality_check':     {'x': 10.0, 'y': 0.0,  'zone': 'Z2'},
    'asrs_input':        {'x': 15.0, 'y': 0.0,  'zone': 'Z2'},
    'asrs_storage':     {'x': 20.0, 'y': 5.0,  'zone': 'Z3'},
    'asrs_output':       {'x': 15.0, 'y': 10.0, 'zone': 'Z4'},
    'staging_area':      {'x': 10.0, 'y': 10.0, 'zone': 'Z4'},
    'outbound_dock':     {'x': 5.0,  'y': 10.0, 'zone': 'Z5'},
    'charging_station':  {'x': 0.0,  'y': 5.0,  'zone': 'Z0'}
}

# Robot fleet with capabilities
ROBOTS = {
    'amr1': {
        'type': 'forklift',
        'capacity_kg': 1000,
        'max_runtime_min': 480,
        'current_battery': 85,
        'current_loc': 'charging_station',
        'capabilities': ['pickup', 'transport', 'deliver', 'stack']
    },
    'amr2': {
        'type': 'forklift',
        'capacity_kg': 1000,
        'max_runtime_min': 480,
        'current_battery': 92,
        'current_loc': 'staging_area',
        'capabilities': ['pickup', 'transport', 'deliver', 'stack']
    },
    'amr3': {
        'type': 'conveyor',
        'capacity_kg': 500,
        'max_runtime_min': 480,
        'current_battery': 78,
        'current_loc': 'asrs_output',
        'capabilities': ['pickup', 'transport', 'deliver']
    }
}

# Task types
TASK_TYPES = ['pickup', 'transport', 'deliver', 'stack', 'unstack', 'recharge']


class CuOptBridge(Node):
    def __init__(self):
        super().__init__('cuopt_bridge')
        
        # Subscribe to orders
        self.order_sub = self.create_subscription(
            String, '/orders', self.order_callback, 10)
        
        # Publish optimized plan
        self.plan_pub = self.create_publisher(String, '/cuopt/plan', 10)
        
        # Publish fleet status
        self.fleet_pub = self.create_publisher(String, '/fleet/status', 10)
        
        self.plan_count = 0
        
        # Publish initial fleet status
        self.publish_fleet_status()
        
        self.get_logger().info('CuOpt Bridge Started (NVIDIA CuOpt Mock)')
        self.get_logger().info(f'Available robots: {list(ROBOTS.keys())}')
        self.get_logger().info(f'Warehouse zones: {len(LOCATIONS)} locations')
        
    def publish_fleet_status(self):
        """Publish current fleet status"""
        status = {
            'timestamp': time.time(),
            'robots': ROBOTS,
            'total_active': sum(1 for r in ROBOTS.values() if r['current_battery'] > 20),
            'total_charging': sum(1 for r in ROBOTS.values() if r['current_battery'] <= 20)
        }
        msg = String()
        msg.data = json.dumps(status)
        self.fleet_pub.publish(msg)
        
    def order_callback(self, msg):
        try:
            order = json.loads(msg.data)
            self.get_logger().info(f'Received order: {order["order_id"]} (priority: {order["priority"]})')
            
            # Create optimized plan (simulating CuOpt)
            plan = self.create_optimized_plan(order)
            
            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            
            self.get_logger().info(
                f'Published optimized plan: {plan["plan_id"]} '
                f'(cost: ${plan["total_cost"]:.2f}, '
                f'est. time: {plan["route"]["estimated_duration_min"]}min)'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to process order: {e}')
            
    def create_optimized_plan(self, order):
        """
        Create an optimized plan simulating NVIDIA CuOpt
        CuOpt considers: travel time, robot availability, task dependencies,
        time windows, robot capabilities, battery constraints
        """
        self.plan_count += 1
        plan_id = f"PLAN-{self.plan_count:03d}"
        
        # Select best robot based on CuOpt-style optimization
        selected_robot = self.select_optimal_robot(order)
        
        # Calculate route based on locations
        tasks = self.optimize_task_sequence(order, selected_robot)
        
        # Calculate metrics
        total_distance = self.calculate_route_distance(tasks)
        estimated_duration = len(tasks) * 2.5  # 2.5 min per task average
        
        return {
            'plan_id': plan_id,
            'order_id': order['order_id'],
            'created_at': time.time(),
            
            # Optimization metadata
            'optimizer': 'NVIDIA CuOpt Mock v1.0',
            'optimization_objective': 'minimize_total_cost',
            'solver_time_ms': random.randint(50, 200),
            
            # Selected robot
            'assigned_robot': {
                'robot_id': selected_robot,
                'type': ROBOTS[selected_robot]['type'],
                'battery_before': ROBOTS[selected_robot]['current_battery']
            },
            
            # Task sequence (optimized)
            'tasks': tasks,
            
            # Route metrics
            'route': {
                'total_distance_m': total_distance,
                'estimated_duration_min': estimated_duration,
                'num_stops': len(tasks),
                'zones_visited': list(set(t['zone'] for t in tasks))
            },
            
            # Cost metrics (CuOpt-style)
            'cost_breakdown': {
                'travel_cost': total_distance * 0.10,
                'labor_cost': estimated_duration * 0.50,
                'energy_cost': total_distance * 0.05,
                'priority_penalty': (6 - order['priority']) * 5.0
            },
            'total_cost': (total_distance * 0.10) + (estimated_duration * 0.50) + 
                         (total_distance * 0.05) + ((6 - order['priority']) * 5.0),
            
            # Constraints satisfied
            'constraints': {
                'time_window_satisfied': True,
                'capacity_satisfied': True,
                'battery_satisfied': True,
                'capability_satisfied': True
            },
            
            'status': 'optimized'
        }
        
    def select_optimal_robot(self, order):
        """
        Simulate CuOpt robot selection based on:
        - Battery level (prefer higher)
        - Proximity to task location
        - Capability match
        - Current workload
        """
        # Simple selection: pick robot with highest battery that's available
        available = [r for r, info in ROBOTS.items() if info['current_battery'] > 20]
        
        if not available:
            return 'amr1'  # Fallback
            
        # Sort by battery (descending)
        available.sort(key=lambda r: ROBOTS[r]['current_battery'], reverse=True)
        
        return available[0]
        
    def optimize_task_sequence(self, order, robot_id):
        """
        Optimize task sequence simulating CuOpt routing
        Returns tasks with time windows and dependencies
        """
        tasks = []
        task_id_base = f"{robot_id}-{self.plan_count}"
        
        # Task 1: Pickup from palletizer
        tasks.append({
            'task_id': f'{task_id_base}-P001',
            'task_type': 'pickup',
            'location': 'palletizer',
            'zone': 'Z1',
            'coords': LOCATIONS['palletizer'],
            'order_id': order['order_id'],
            'items': order['items'],
            'quantities': order['quantities'],
            'weight_kg': sum(order['quantities']),
            'sequence': 1,
            'dependency': None,
            'time_window': {
                'earliest': time.time() + 60,
                'latest': time.time() + 300
            },
            'service_time_min': 2.0
        })
        
        # Task 2: Transport to ASRS input
        tasks.append({
            'task_id': f'{task_id_base}-T001',
            'task_type': 'transport',
            'location': 'asrs_input',
            'zone': 'Z2',
            'coords': LOCATIONS['asrs_input'],
            'order_id': order['order_id'],
            'sequence': 2,
            'dependency': f'{task_id_base}-P001',
            'time_window': {
                'earliest': time.time() + 180,
                'latest': time.time() + 420
            },
            'service_time_min': 1.0
        })
        
        # Task 3: Store in ASRS
        tasks.append({
            'task_id': f'{task_id_base}-S001',
            'task_type': 'stack',
            'location': 'asrs_storage',
            'zone': 'Z3',
            'coords': LOCATIONS['asrs_storage'],
            'order_id': order['order_id'],
            'sequence': 3,
            'dependency': f'{task_id_base}-T001',
            'time_window': {
                'earliest': time.time() + 300,
                'latest': time.time() + 600
            },
            'service_time_min': 3.0,
            'asrs_bin': f'BIN-{random.randint(1, 50):03d}'
        })
        
        return tasks
        
    def calculate_route_distance(self, tasks):
        """Calculate total route distance"""
        total = 0.0
        prev_loc = ROBOTS['amr1']['current_loc']  # Start from robot location
        
        for task in tasks:
            curr_loc = task['location']
            if prev_loc in LOCATIONS and curr_loc in LOCATIONS:
                dx = LOCATIONS[curr_loc]['x'] - LOCATIONS[prev_loc]['x']
                dy = LOCATIONS[curr_loc]['y'] - LOCATIONS[prev_loc]['y']
                total += (dx**2 + dy**2) ** 0.5
            prev_loc = curr_loc
            
        return round(total, 2)


def main(args=None):
    rclpy.init(args=args)
    node = CuOptBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
