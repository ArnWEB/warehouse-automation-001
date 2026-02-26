#!/usr/bin/env python3
"""
Fleet Execution Visualizer
- Shows the complete flow: Tasks → Fleet Manager → CuOpt → Executors
- Displays in terminal like a graph
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
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


class FleetVisualizer(Node):
    def __init__(self):
        super().__init__('fleet_visualizer')
        
        self.get_logger().info('Fleet Visualizer starting...')
        self.get_logger().info('Subscribing to: /fleet/tasks, /fleet/cuopt_plan')
        
        self.tasks_sub = self.create_subscription(
            Int32MultiArray, '/fleet/tasks', self.tasks_callback, 10)
        
        self.plan_sub = self.create_subscription(
            String, '/fleet/cuopt_plan', self.plan_callback, 10)
        
        self.debug_subs = {}
        for robot_id in ['amr1', 'amr2', 'amr3']:
            self.debug_subs[robot_id] = self.create_subscription(
                String, f'/{robot_id}/executor_debug',
                lambda msg, rid=robot_id: self.debug_callback(msg, rid), 10)
        
        self.plan_count = 0
        self.current_tasks = []
        self.robot_status = {r: 'idle' for r in ['amr1', 'amr2', 'amr3']}
        
        self.timer = self.create_timer(2.0, self.print_dashboard)
        
        self.get_logger().info('Fleet Visualizer started!')

    def tasks_callback(self, msg):
        self.current_tasks = list(msg.data)
        self.get_logger().info(f'📥 NEW TASKS: {self.current_tasks}')

    def plan_callback(self, msg):
        self.plan_count += 1
        plan = json.loads(msg.data)
        
        print("\n" + "="*60)
        print(f"📦 PLAN #{self.plan_count} - CuOpt Optimization Complete")
        print("="*60)
        
        for robot_id, assignment in plan.get('assignments', {}).items():
            tasks = assignment.get('tasks', [])
            task_names = [WAREHOUSE_LOCATIONS[t]['name'] for t in tasks if t in WAREHOUSE_LOCATIONS]
            print(f"  🤖 {robot_id} → {task_names}")
        
        print("="*60 + "\n")

    def debug_callback(self, msg, robot_id):
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            if state == 'moving':
                loc = data.get('location', '')
                self.robot_status[robot_id] = f"moving to {loc}"
        except:
            pass

    def print_dashboard(self):
        print("\n" + "─"*60)
        print("🚗 WAREHOUSE FLEET STATUS")
        print("─"*60)
        
        print(f"📋 Pending Tasks: {self.current_tasks}")
        print()
        
        for robot_id in ['amr1', 'amr2', 'amr3']:
            status = self.robot_status[robot_id]
            print(f"  🤖 {robot_id}: {status}")
        
        print("─"*60)
        
        print("\n📍 Warehouse Locations:")
        for loc_id, loc in WAREHOUSE_LOCATIONS.items():
            marker = "⏳"
            if self.current_tasks and loc_id in self.current_tasks:
                marker = "📌"
            print(f"  {marker} {loc_id}: {loc['name']:15s} ({loc['x']:5.1f}, {loc['y']:5.1f})")


def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
