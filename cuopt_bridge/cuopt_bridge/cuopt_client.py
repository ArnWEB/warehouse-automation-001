#!/usr/bin/env python3
"""
CuOpt Client Node
- Subscribes to /cuopt/order_data (JSON with transport orders)
- Calls NVIDIA cuOpt for optimization
- Publishes optimized plans to /fleet/cuopt_plan

New flow based on NVIDIA CuOpt documentation:
- Input: pickup_location, delivery_location, order_demand, time windows
- Uses pickup_delivery_pairs for same-robot constraint
- Full CuOpt SDK implementation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time
import random
import numpy as np

from cuopt_sh_client import CuOptServiceSelfHostClient
from cuopt import routing
from cuopt import distance_engine
import cudf
import numpy as np

CUOPT_SERVER_IP = "localhost"
CUOPT_SERVER_PORT = 5000

WAREHOUSE_LOCATIONS = {
    0: {"name": "depot", "x": 0.0, "y": 0.0},
    1: {"name": "station_1", "x": 5.0, "y": 10.0},
    2: {"name": "station_2", "x": 10.0, "y": 5.0},
    3: {"name": "station_3", "x": 15.0, "y": 10.0},
    4: {"name": "processing_4", "x": 20.0, "y": 5.0},
    5: {"name": "processing_5", "x": 25.0, "y": 10.0},
    6: {"name": "processing_6", "x": 30.0, "y": 5.0},
    7: {"name": "station_7", "x": 35.0, "y": 10.0},
    8: {"name": "station_8", "x": 40.0, "y": 5.0},
    9: {"name": "station_9", "x": 45.0, "y": 10.0},
}

WAYPOINT_GRAPH = {
    "0": {
        "offsets": [0, 1, 3, 7, 9, 11, 13, 15, 17, 20, 22],
        "edges": [2, 2, 4, 0, 1, 3, 5, 2, 6, 1, 7, 2, 8, 3, 9, 4, 8, 5, 7, 9, 6, 8],
        "weights":  [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 2, 1, 2, 2, 1, 2]
    }
}

TARGET_LOCATIONS = np.array([0, 4, 5, 6])


class CuOptClient(Node):
    def __init__(self, use_mock: bool = True):
        super().__init__("cuopt_client")

        # self.use_mock = use_mock or not CUOPT_SDK_AVAILABLE
        self.use_mock = False

        if self.use_mock:
            self.get_logger().info("CuOpt Client: Using MOCK solver")
        else:
            server = f"{CUOPT_SERVER_IP}:{CUOPT_SERVER_PORT}"
            self.get_logger().info(f"CuOpt Client: Using REAL cuOpt API at {server}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.order_sub = self.create_subscription(
            String, "/cuopt/order_data", self.order_data_callback, qos
        )

        self.robot_state_sub = self.create_subscription(
            String, "/fleet/robot_states", self.robot_states_callback, qos
        )

        self.plan_pub = self.create_publisher(String, "/fleet/cuopt_plan", qos)

        self.plan_pub_debug = self.create_publisher(
            String, "/fleet/cuopt_plan_debug", 10
        )

        self.transport_orders = []
        self.fleet_config = {
            "vehicle_locations": [[0, 0], [0, 0], [0, 0]],
            "capacities": [[10], [10], [10]]
        }
        self.solver_config = {
            "time_limit": 5
        }

        self.robot_states = {}
        self.plan_count = 0

        self.get_logger().info("CuOpt Client Node initialized")
        self.get_logger().info("Subscribing to /cuopt/order_data for transport orders")

    def robot_states_callback(self, msg):
        try:
            states = json.loads(msg.data)
            self.robot_states = {r["robot_id"]: r for r in states}
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot states: {e}")

    def order_data_callback(self, msg):
        try:
            data = json.loads(msg.data)

            self.get_logger().info(f"Received order data:\n{json.dumps(data, indent=4)}")
            
            if "transport_orders" in data:
                self.transport_orders = data["transport_orders"]
            
            if "fleet_data" in data:
                self.fleet_config = data["fleet_data"]
            
            if "solver_config" in data:
                self.solver_config = data["solver_config"]

            self.get_logger().info(
                f"Received {len(self.transport_orders)} transport orders"
            )

            if self.transport_orders:
                self.solve_and_publish()

        except Exception as e:
            self.get_logger().error(f"Failed to parse order data: {e}")

    def solve_and_publish(self):
        if not self.transport_orders:
            self.get_logger().warn("No transport orders to optimize")
            return

        self.plan_count += 1

        if self.use_mock:
            plan = self._solve_mock()
        else:
            plan = self._solve_cuopt()

        msg = String()
        msg.data = json.dumps(plan)
        self.plan_pub.publish(msg)
        self.get_logger().info(f"Published plan to /fleet/cuopt_plan:\n{json.dumps(plan, indent=4)}")

        debug_msg = String()
        debug_msg.data = json.dumps(
            {
                "plan_id": plan["plan_id"],
                "robot_assignments": {      
                    k: v.get("tasks", []) for k, v in plan.get("assignments", {}).items()
                },
                "timestamp": time.time(),
            }
        )
        self.plan_pub_debug.publish(debug_msg)

        self.get_logger().info(
            f"Plan {plan['plan_id']}: "
            + ", ".join([f"{k}→{v.get('tasks', [])}" for k, v in plan.get("assignments", {}).items()])
        )
        self.get_logger().info(
            f"Routes {plan['plan_id']}: "
            + ", ".join([f"{k}→{v.get('route', [])}" for k, v in plan.get("assignments", {}).items()])
        )

        self.transport_orders = []

    def _solve_mock(self):
        n_robots = len(self.robot_states) if self.robot_states else 2
        n_orders = len(self.transport_orders)

        assignments = {}
        tasks_per_robot = [[] for _ in range(n_robots)]

        for i in range(n_orders):
            robot_idx = i % n_robots
            tasks_per_robot[robot_idx].append(i)

        robot_ids = list(self.robot_states.keys()) if self.robot_states else ["amr1", "amr2", "amr3"]

        for idx in range(min(n_robots, len(robot_ids))):
            robot_id = robot_ids[idx]
            assignments[robot_id] = {
                "tasks": tasks_per_robot[idx],
                "route": [],
                "total_cost": random.uniform(10, 50),
                "estimated_time": len(tasks_per_robot[idx]) * 2.5,
            }

        order_mapping = {}
        for robot_id, data in assignments.items():
            for task_idx in data["tasks"]:
                order_mapping[str(task_idx)] = robot_id

        return {
            "plan_id": self.plan_count,
            "created_at": time.time(),
            "solver": "mock",
            "solve_time_ms": random.uniform(50, 200),
            "assignments": assignments,
            "order_mapping": order_mapping,
            "total_cost": sum(a["total_cost"] for a in assignments.values()),
            "num_orders": n_orders,
        }

    def _solve_cuopt(self):
        try:
            transport_orders = self.transport_orders
            print(f"transport_orders: {transport_orders}")
            n_orders = len(transport_orders)

            pickup_locations = [order.get("pickup_location", 0) for order in transport_orders]
            delivery_locations = [order.get("delivery_location", 0) for order in transport_orders]
            order_demands = [order.get("order_demand", 1) for order in transport_orders]

            earliest_pickup = [order.get("earliest_pickup", 0) for order in transport_orders]
            latest_pickup = [order.get("latest_pickup", 100) for order in transport_orders]
            pickup_service_time = [order.get("pickup_service_time", 2) for order in transport_orders]

            earliest_delivery = [order.get("earliest_delivery", 0) for order in transport_orders]
            latest_delivery = [order.get("latest_delivery", 100) for order in transport_orders]
            delivery_service_time = [order.get("delivery_service_time", 2) for order in transport_orders]

            waypoint_graph_data = WAYPOINT_GRAPH

            waypoint_graph = distance_engine.WaypointMatrix(
                np.array(waypoint_graph_data["0"]["offsets"]),
                np.array(waypoint_graph_data["0"]["edges"]),
                np.array(waypoint_graph_data["0"]["weights"])
            )

            target_locations = TARGET_LOCATIONS
            cost_matrix = waypoint_graph.compute_cost_matrix(target_locations)
            transit_time_matrix = cost_matrix.copy(deep=True)

            target_map = {v: k for k, v in enumerate(target_locations)}
            index_map = {k: v for k, v in enumerate(target_locations)}

            pickup_indices = [target_map.get(loc, 0) for loc in pickup_locations]
            delivery_indices = [target_map.get(loc, 0) for loc in delivery_locations]

            pickup_order_locations = cudf.Series(pickup_indices)
            delivery_order_locations = cudf.Series(delivery_indices)
            order_locations = cudf.concat([pickup_order_locations, delivery_order_locations], ignore_index=True)

            raw_demand = cudf.Series(order_demands)
            drop_off_demand = raw_demand * -1
            order_demand = cudf.concat([raw_demand, drop_off_demand], ignore_index=True)

            print(f"order_demand: {order_demand}")
            print(f"order_locations: {order_locations}")

            fleet_data = self.fleet_config
            # vehicle_locations = fleet_data.get("vehicle_locations", [[0, 0], [0, 0]])
            vehicle_locations = [[0, 0], [0, 0]]
            capacities = fleet_data.get("capacities", [[10], [10]])

            n_locations = len(cost_matrix)
            n_vehicles = len(vehicle_locations)
            print(f"Number of vehicles: {n_vehicles}")
            print(f"Vehicle locations: {vehicle_locations}")
            print(f"Capacities: {capacities}")
            n_orders_total = n_orders * 2

            data_model = routing.DataModel(n_locations, n_vehicles, n_orders_total)
            data_model.add_cost_matrix(cost_matrix)
            data_model.add_transit_time_matrix(transit_time_matrix)

            robot_data = {
                "robot_ids": [i for i in range(2)],
                "carrying_capacity":[2,2]
            }
            robot_data = cudf.DataFrame(robot_data).set_index('robot_ids')
            # robot_data
            

            print(f"robot_data: {robot_data}")
            # capacities[0] if capacities else [10]}).set_index("capacity").index

            data_model.add_capacity_dimension(
                "demand", order_demand, robot_data['carrying_capacity']
            )

            print("here2")

            data_model.set_order_locations(order_locations)

            print("here3")

            npair_orders = n_orders
            pickup_orders = cudf.Series([i for i in range(npair_orders)])
            delivery_orders = cudf.Series([i + npair_orders for i in range(npair_orders)])
            data_model.set_pickup_delivery_pairs(pickup_orders, delivery_orders)

            order_time_window_earliest = cudf.concat([
                cudf.Series(earliest_pickup),
                cudf.Series(earliest_delivery)
            ], ignore_index=True)

            order_time_window_latest = cudf.concat([
                cudf.Series(latest_pickup),
                cudf.Series(latest_delivery)
            ], ignore_index=True)

            order_service_time = cudf.concat([
                cudf.Series(pickup_service_time),
                cudf.Series(delivery_service_time)
            ], ignore_index=True)

            data_model.set_order_time_windows(order_time_window_earliest, order_time_window_latest)
            data_model.set_order_service_times(order_service_time)

            vehicle_earliest_time = cudf.Series([0] * n_vehicles)
            vehicle_latest_time = cudf.Series([1000] * n_vehicles)
            data_model.set_vehicle_time_windows(vehicle_earliest_time, vehicle_latest_time)

            solver_settings = routing.SolverSettings()
            solver_settings.set_time_limit(self.solver_config.get("time_limit", 5))

            self.get_logger().info("Sending problem to CuOpt solver...")


            self.get_logger().info(f"Data model: {data_model}")
            self.get_logger().info(f"Solver settings: {solver_settings}")

            print(f"Data model: {data_model}")
            print(f"Solver settings: {solver_settings}")
            routing_solution = routing.Solve(data_model, solver_settings)

            print(f"Routing solution: \n {routing_solution.route}")

            if routing_solution.get_status() != 0:
                self.get_logger().warn(f"CuOpt failed with status: {routing_solution.get_status()}")
                return self._solve_mock()

            self.get_logger().info("CuOpt solver completed successfully")



            solution_cost = routing_solution.get_total_objective()
            vehicle_count = routing_solution.get_vehicle_count()

            self.get_logger().info(
                f"CuOpt optimization complete: {vehicle_count} vehicles, cost: {solution_cost}"
            )

            # route_df = routing_solution.route

            target_loc_route = [
                index_map[loc]
                for loc in routing_solution.route["location"].to_arrow().to_pylist()
            ]
            routing_solution.route["order_array_index"] = routing_solution.route["route"]
            routing_solution.route["route"] = target_loc_route
            print(f"route_df_mapped_with_waypoints_graph: \n\n\n{routing_solution.route}\n\n\n")

            assignments = {}
            order_mapping = {}
            unique_robot_ids = routing_solution.route["truck_id"].unique()
            all_routes = routing_solution.get_route()

            for robot in unique_robot_ids.to_arrow().to_pylist():
                route = all_routes[all_routes["truck_id"] == robot]
                waypoint_route = waypoint_graph.compute_waypoint_sequence(
                    target_locations, route
                )
                print(
                    f"Target location level route for robot {robot}:\n{all_routes[all_routes['truck_id'] == robot]['route']}\n\n"
                )
                print(f"Waypoint level route for robot {robot}:\n{waypoint_route}\n\n")

                assignments[f"amr{robot+1}"] = {
                    "tasks": waypoint_route["waypoint_type"].to_arrow().to_pylist(),
                    "route": waypoint_route["waypoint_sequence"].to_arrow().to_pylist(),
                    "total_cost": 0.0,
                    "estimated_time": 0.0
                }


            return {
                "plan_id": self.plan_count,
                "created_at": time.time(),
                "solver": "cuopt",
                "solve_time_ms": 0.0,
                "assignments": assignments,
                # "order_mapping": order_mapping,
                "total_cost": solution_cost,
                "num_orders": n_orders,
            }

        except Exception as e:
            self.get_logger().error(f"CuOpt solver failed: {type(e).__name__}: {e}")
            self.get_logger().info("Falling back to mock solver")
            return self._solve_mock()


def main(args=None):
    rclpy.init(args=args)
    node = CuOptClient(use_mock=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
