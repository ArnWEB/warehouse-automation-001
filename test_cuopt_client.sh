#!/usr/bin/env bash
# Run cuopt_client and test order

cd ~/warehouse-automation-001
source /opt/ros/humble/setup.bash
source ~/warehouse-automation-001/install/setup.bash

# Run cuopt_client in background and capture output
ros2 run cuopt_bridge cuopt_client > /tmp/cuopt_client.log 2>&1 &
CLIENT_PID=$!

echo "Waiting for cuopt_client to start..."
sleep 3

echo "Publishing test order..."
ros2 topic pub /cuopt/order_data std_msgs/String '{data: "{\"transport_orders\": [{\"pickup_location\": 4, \"delivery_location\": 5, \"order_demand\": 1}], \"fleet_data\": {\"vehicle_locations\": [[0, 0]], \"capacities\": [[10]]}}"}' --once

echo "Waiting for result..."
sleep 5

echo "Checking /fleet/cuopt_plan..."
ros2 topic echo /fleet/cuopt_plan --once

echo "Stopping cuopt_client..."
kill $CLIENT_PID 2>/dev/null

echo "Done!"
