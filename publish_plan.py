#!/usr/bin/env python3
"""
CuOpt Plan Publisher for Multi-Robot Coordination
Publishes plan multiple times to ensure all subscribers receive it
"""

import json
import time
import sys
import subprocess

def publish_plan_multiple_times(num_publishes=5, delay_between=0.5):
    """Publish the plan multiple times to ensure all subscribers get it"""
    
    plan_json = {
        "plan_id": 1,
        "assignments": {
            "amr1": {"tasks": [10, 20, 30], "total_cost": 15.5},
            "amr2": {"tasks": [40, 50, 60], "total_cost": 18.3},
            "amr3": {"tasks": [70, 80, 90], "total_cost": 20.1}
        },
        "total_cost": 54.0
    }
    
    plan_str = json.dumps(plan_json)
    
    for i in range(num_publishes):
        cmd = f"""ros2 topic pub /fleet/cuopt_plan std_msgs/String "data: '{plan_str}'" --once"""
        print(f"[Publish {i+1}/{num_publishes}] Publishing CuOpt plan...")
        subprocess.run(cmd, shell=True, capture_output=True)
        if i < num_publishes - 1:
            time.sleep(delay_between)
    
    print(f"✓ Published plan {num_publishes} times")

if __name__ == "__main__":
    publish_plan_multiple_times(num_publishes=5, delay_between=0.3)
