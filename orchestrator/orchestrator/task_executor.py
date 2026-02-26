#!/usr/bin/env python3
"""
Task Executor Node (Per Robot)
- Executes CuOpt plans using Nav2
- Handles plan updates and replanning
- Smart replan: doesn't restart if goal unchanged

State Machine:
  WAIT_FOR_PLAN → EXECUTE_GOAL → WAIT_NAV2 → NEXT_GOAL
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
import json
import time
import math


WAREHOUSE_LOCATIONS = {
    0: {'name': 'charging', 'x': -5.0, 'y': 0.0, 'theta': 0.0},
    1: {'name': 'palletizer', 'x': -8.0, 'y': 3.0, 'theta': 0.0},
    2: {'name': 'quality_check', 'x': -4.0, 'y': 3.0, 'theta': 0.0},
    3: {'name': 'asrs_in', 'x': 8.0, 'y': 3.0, 'theta': 0.0},
    4: {'name': 'asrs_store', 'x': 8.0, 'y': 5.0, 'theta': 0.0},
    5: {'name': 'asrs_out', 'x': 8.0, 'y': 7.0, 'theta': 0.0},
    6: {'name': 'staging', 'x': 0.0, 'y': -5.0, 'theta': 0.0},
    7: {'name': 'outbound', 'x': 5.0, 'y': -5.0, 'theta': 0.0},
}


class TaskExecutor(Node):
    def __init__(self, robot_id: str = 'amr1', use_nav2: bool = False):
        super().__init__(f'task_executor_{robot_id}')
        
        self.robot_id = robot_id
        self.use_nav2 = use_nav2
        
        self.current_x = -5.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.current_plan = []
        self.current_goal_index = 0
        self.plan_id = -1
        self.replan_requested = False
        self.executing = False
        
        self.plan_sub = self.create_subscription(
            String,
            '/fleet/cuopt_plan',
            self.plan_callback,
            10)
        
        self.status_pub = self.create_publisher(Int32, f'/{robot_id}/status', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{robot_id}/odom', 10)
        self.debug_pub = self.create_publisher(String, f'/{robot_id}/executor_debug', 10)
        
        self.timer = self.create_timer(0.1, self.update_position)
        
        self.get_logger().info(f'Task Executor initialized for {robot_id}')

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            assignments = plan.get('assignments', {})
            
            robot_assignments = {
                'amr1': 0,
                'amr2': 1,
                'amr3': 2,
            }
            
            idx = robot_assignments.get(self.robot_id, 0)
            
            if idx in assignments:
                new_tasks = assignments[idx]['tasks']
                new_plan_id = plan['plan_id']
                
                if new_plan_id != self.plan_id:
                    self.get_logger().info(
                        f'NEW PLAN {new_plan_id}: {self.robot_id} → {new_tasks}'
                    )
                    
                    if self.executing:
                        self.handle_replan(new_tasks, new_plan_id)
                    else:
                        self.current_plan = new_tasks
                        self.current_goal_index = 0
                        self.plan_id = new_plan_id
                        
                        self.execute_plan()
                else:
                    self.get_logger().debug(f'Same plan {new_plan_id}, ignoring')
                    
        except Exception as e:
            self.get_logger().error(f'Failed to parse plan: {e}')

    def handle_replan(self, new_tasks, new_plan_id):
        current_goal = self.current_plan[self.current_goal_index] if self.current_goal_index < len(self.current_plan) else -1
        
        if current_goal in new_tasks[:self.current_goal_index + 1]:
            self.get_logger().info(f'Smart replan: continuing to goal {current_goal}')
            self.replan_requested = False
        else:
            self.get_logger().info(f'REPLAN: canceling goal {current_goal}, new route: {new_tasks}')
            self.replan_requested = True
            self.current_plan = new_tasks
            self.current_goal_index = 0
            self.plan_id = new_plan_id
            
            self.execute_plan()

    def execute_plan(self):
        if not self.current_plan:
            self.get_logger().info('No tasks in plan, waiting...')
            self.executing = False
            self.publish_status(0)
            return
        
        self.executing = True
        self.publish_status(self.current_plan[self.current_goal_index])
        
        while self.current_goal_index < len(self.current_plan):
            task_id = self.current_plan[self.current_goal_index]
            
            self.get_logger().info(
                f'EXECUTING: {self.robot_id} → Task {task_id} @ {WAREHOUSE_LOCATIONS[task_id]["name"]}'
            )
            
            self.publish_debug({
                'state': 'moving',
                'task_id': task_id,
                'location': WAREHOUSE_LOCATIONS[task_id]['name'],
                'progress': f'{self.current_goal_index + 1}/{len(self.current_plan)}',
                'plan_id': self.plan_id
            })
            
            target = WAREHOUSE_LOCATIONS[task_id]
            self.move_to(target['x'], target['y'])
            
            self.current_goal_index += 1
            
            if self.replan_requested:
                self.get_logger().info('Replan received during execution, restarting...')
                self.replan_requested = False
                break
        
        if self.current_goal_index >= len(self.current_plan):
            self.get_logger().info(f'PLAN COMPLETE: {self.robot_id} finished all tasks!')
            self.executing = False
            self.publish_status(0)

    def move_to(self, target_x, target_y):
        speed = 0.5
        arrived = False
        
        while not arrived:
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < 0.1:
                arrived = True
                break
            
            angle = math.atan2(dy, dx)
            
            self.current_x += math.cos(angle) * speed * 0.1
            self.current_y += math.sin(angle) * speed * 0.1
            self.current_theta = angle
            
            time.sleep(0.1)

    def update_position(self):
        from geometry_msgs.msg import Quaternion
        
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = self.robot_id
        
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0
        
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.current_theta / 2)
        q.w = math.cos(self.current_theta / 2)
        odom.pose.pose.orientation = q
        
        self.odom_pub.publish(odom)

    def publish_status(self, task_id: int):
        msg = Int32()
        msg.data = task_id
        self.status_pub.publish(msg)

    def publish_debug(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor(robot_id='amr1', use_nav2=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
