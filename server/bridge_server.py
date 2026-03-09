#!/usr/bin/env python3
"""
AMR Bridge Server - FastAPI REST API for AMR UI
Connects UI to ROS2 CuOpt system

Usage:
    uv add fastapi uvicorn websockets rclpy
    python server/bridge_server.py
"""

import json
import threading
from typing import Dict, List, Optional
from datetime import datetime

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from pydantic import BaseModel
import uvicorn
import logging

logger = logging.getLogger(__name__)

graph = {
    0:{
        "edges":[2], 
        "weights":[2]},
    1:{
        "edges":[2, 4], 
        "weights":[2, 2]},
    2:{
        "edges":[0, 1, 3, 5], 
        "weights":[2, 2, 2, 2]},
    3:{
        "edges":[2, 6], 
        "weights":[2, 2]},
    4:{
        "edges":[1, 7], 
        "weights":[2, 1]},
    5:{
        "edges":[2, 8], 
        "weights":[2, 1]},
    6:{
        "edges":[3, 9], 
        "weights":[2, 1]},
    7:{
        "edges":[4, 8], 
        "weights":[1, 2]},
    8:{
        "edges":[5, 7, 9], 
        "weights":[1, 2, 2]},
    9:{
        "edges":[6, 8], 
        "weights":[1, 2]}
}

WAREHOUSE_NODES = {
    0: {"name": "depot", "x": 0.0, "y": 0.0, "type": "depot", "can_pickup": False, "can_deliver": True},
    1: {"name": "station_1", "x": 5.0, "y": 10.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
    2: {"name": "station_2", "x": 10.0, "y": 5.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
    3: {"name": "station_3", "x": 15.0, "y": 10.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
    4: {"name": "processing_4", "x": 20.0, "y": 5.0, "type": "processing", "can_pickup": True, "can_deliver": True},
    5: {"name": "processing_5", "x": 25.0, "y": 10.0, "type": "processing", "can_pickup": True, "can_deliver": True},
    6: {"name": "processing_6", "x": 30.0, "y": 5.0, "type": "processing", "can_pickup": True, "can_deliver": True},
    7: {"name": "station_7", "x": 35.0, "y": 10.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
    8: {"name": "station_8", "x": 40.0, "y": 5.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
    9: {"name": "station_9", "x": 45.0, "y": 10.0, "type": "incoming", "can_pickup": True, "can_deliver": False},
}





FLEET_CONFIG = {
    "vehicle_locations": [[0, 0], [0, 0], [0, 0]],
    "capacities": [[10], [10], [10]]
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    start_ros_thread()
    yield


app = FastAPI(title="AMR Bridge Server", version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class TransportOrder(BaseModel):
    pickup_location: int
    delivery_location: int
    order_demand: int = 1
    earliest_pickup: int = 0
    latest_pickup: int = 100
    pickup_service_time: int = 2
    earliest_delivery: int = 0
    latest_delivery: int = 100
    delivery_service_time: int = 2


class CuOptRequest(BaseModel):
    transport_orders: List[TransportOrder]
    fleet_data: Optional[Dict] = None
    solver_config: Optional[Dict] = None


class FleetConfigRequest(BaseModel):
    vehicle_locations: List[List[int]]
    capacities: List[List[int]]


connected_clients: List[WebSocket] = []

latest_robot_states: Dict = {}
latest_cuopt_plan: Dict = {}
ros_thread: Optional[threading.Thread] = None
global_publish_order = None


def init_ros():
    """Initialize ROS2 connection in a separate thread"""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        
        if not rclpy.ok():
            rclpy.init()
        
        class BridgeNode(Node):
            def __init__(self):
                super().__init__('bridge_server')
                
        node = BridgeNode()
        
        def robot_state_callback(msg):
            global latest_robot_states
            try:
                states = json.loads(msg.data)
                latest_robot_states = {r["robot_id"]: r for r in states}
                broadcast_to_clients({
                    "type": "robot_states",
                    "data": latest_robot_states
                })
            except Exception as e:
                print(f"Error parsing robot states: {e}")
        
        def plan_callback(msg):
            global latest_cuopt_plan
            try:
                plan = json.loads(msg.data)
                latest_cuopt_plan = plan
                broadcast_to_clients({
                    "type": "cuopt_plan",
                    "data": plan
                })
            except Exception as e:
                print(f"Error parsing cuopt plan: {e}")
        
        node.create_subscription(String, '/fleet/robot_states', robot_state_callback, 10)
        node.create_subscription(String, '/fleet/cuopt_plan', plan_callback, 10)
        
        plan_pub = node.create_publisher(String, '/cuopt/order_data', 10)
        
        def publish_order(data: dict):
            logger.info(f"Publishing order: {data}")
            msg = String()
            msg.data = json.dumps(data)
            print(f"Publishing order: {msg.data}")
            plan_pub.publish(msg)
        
        node.publish_order = publish_order
        
        global global_publish_order
        global_publish_order = publish_order
        
        print("ROS2 node initialized successfully")
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except Exception as e:
        print(f"ROS2 initialization error: {e}")


def start_ros_thread():
    global ros_thread
    if ros_thread is None or not ros_thread.is_alive():
        ros_thread = threading.Thread(target=init_ros, daemon=True)
        ros_thread.start()


async def broadcast_to_clients(message: dict):
    """Broadcast message to all connected WebSocket clients"""
    disconnected = []
    for client in connected_clients:
        try:
            await client.send_json(message)
        except:
            disconnected.append(client)
    
    for client in disconnected:
        connected_clients.remove(client)


@app.get("/")
async def root():
    return {
        "name": "AMR Bridge Server",
        "version": "1.0.0",
        "status": "running"
    }


@app.get("/api/nodes")
async def get_nodes():
    """Get all available warehouse nodes"""
    return {
        "nodes": WAREHOUSE_NODES,
        "pickup_nodes": [k for k, v in WAREHOUSE_NODES.items() if v["can_pickup"]],
        "delivery_nodes": [k for k, v in WAREHOUSE_NODES.items() if v["can_deliver"]]
    }


@app.get("/api/nodes/pickup")
async def get_pickup_nodes():
    """Get nodes available for pickup"""
    return {
        "nodes": {k: v for k, v in WAREHOUSE_NODES.items() if v["can_pickup"]}
    }


@app.get("/api/nodes/delivery")
async def get_delivery_nodes():
    """Get nodes available for delivery"""
    return {
        "nodes": {k: v for k, v in WAREHOUSE_NODES.items() if v["can_deliver"]}
    }


@app.get("/api/waypoint-graph")
async def get_waypoint_graph():
    """
    Get the complete waypoint graph with edges, weights, and node types.
    Returns the full graph structure for visualization and navigation.
    """
    return {
        "nodes": WAREHOUSE_NODES,
        "graph": graph,
        "node_count": len(WAREHOUSE_NODES),
        "edge_count": sum(len(data["edges"]) for data in graph.values())
    }


@app.post("/api/orders")
async def submit_orders(request: CuOptRequest):
    """Submit transport orders to CuOpt for optimization"""
    
    order_dict = {
        "transport_orders": [order.model_dump() for order in request.transport_orders],
        "fleet_data": request.fleet_data or FLEET_CONFIG,
        "solver_config": request.solver_config or {"time_limit": 5}
    }

    print("herer1")
    
    try:
        print("herer2")
        
        if global_publish_order:
            print("herer3")
            global_publish_order(order_dict)
        else:
            raise Exception("ROS2 publisher not initialized yet")
        
        return {
            "status": "success",
            "message": f"Submitted {len(request.transport_orders)} orders to CuOpt",
            "orders": order_dict
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/plan")
async def get_latest_plan():
    """Get the latest CuOpt optimization plan"""
    if latest_cuopt_plan:
        return latest_cuopt_plan
    return {"message": "No plan available yet"}


@app.get("/api/fleet/status")
async def get_fleet_status():
    """Get current fleet status"""
    return {
        "robots": latest_robot_states,
        "last_update": datetime.now().isoformat()
    }


@app.get("/api/fleet/positions")
async def get_fleet_positions():
    """Get robot positions for UI visualization"""
    positions = []
    for robot_id, state in latest_robot_states.items():
        positions.append({
            "robot_id": robot_id,
            "x": state.get("x", 0),
            "y": state.get("y", 0),
            "theta": state.get("theta", 0),
            "busy": state.get("busy", False),
            "current_task": state.get("current_task", -1),
            "progress": state.get("progress", 0)
        })
    return {"robots": positions}


@app.get("/api/fleet/config")
async def get_fleet_config():
    """Get current fleet configuration"""
    return FLEET_CONFIG


@app.post("/api/fleet/config")
async def update_fleet_config(config: FleetConfigRequest):
    """Update fleet configuration"""
    global FLEET_CONFIG
    FLEET_CONFIG = {
        "vehicle_locations": config.vehicle_locations,
        "capacities": config.capacities
    }
    return {"status": "success", "config": FLEET_CONFIG}


@app.websocket("/ws/fleet")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time fleet updates"""
    await websocket.accept()
    connected_clients.append(websocket)
    
    try:
        await websocket.send_json({
            "type": "connected",
            "message": "Connected to AMR Fleet WebSocket"
        })
        
        if latest_robot_states:
            await websocket.send_json({
                "type": "robot_states",
                "data": latest_robot_states
            })
        
        if latest_cuopt_plan:
            await websocket.send_json({
                "type": "cuopt_plan",
                "data": latest_cuopt_plan
            })
        
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                
                if message.get("type") == "ping":
                    await websocket.send_json({"type": "pong"})
                    
            except json.JSONDecodeError:
                pass
                
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "ros_connected": ros_thread is not None and ros_thread.is_alive(),
        "websocket_clients": len(connected_clients)
    }


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
