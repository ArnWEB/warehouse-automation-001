# AGENTS.md - Warehouse Automation Project

## Project Overview

ROS 2 Humble warehouse automation with packages:
- `order_system` - Order generation and listening
- `cuopt_bridge` - Mock NVIDIA CuOpt optimizer
- `orchestrator` - Task execution and robot management
- `amr_description` - Robot URDF, Gazebo world, Isaac Sim scripts
- `warehouse_msgs` - Custom message definitions
- `hello_world_pkgs` - Example packages

## Build Commands
```bash
source /opt/ros/humble/setup.bash
colcon build
colcon build --packages-select <package>
colcon build --packages-select <package> --symlink-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Lint Commands
```bash
colcon test
cd <package> && ament_flake8 . && ament_pep257 .
python3 -m pytest test/test_flake8.py -v
python3 -m pytest test/test_pep257.py -v
python3 -m pytest test/test_copyright.py -v
python3 -m pytest test/test_flake8.py::test_flake8 -v
```

## Test Commands
```bash
colcon test
colcon test --packages-select <package>
colcon test --event-handlers console_direct+
```

## Code Style Guidelines

### General
- Python 3, follow PEP 8 (flake8), max line length 100
- Shebang `#!/usr/bin/env python3` for executables

### Imports (order: stdlib → third-party → ROS messages → ROS nodes)
```python
#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

### Naming Conventions
- **Classes**: PascalCase (`OrderGenerator`)
- **Functions/methods**: snake_case (`generate_order`)
- **Variables**: snake_case (`order_count`)
- **Constants**: UPPER_SNAKE_CASE (`MAX_RETRIES`)
- **ROS topics/services**: snake_case with slashes (`/orders`, `/robot_status`)

### Type Annotations
```python
from typing import List, Dict, Optional

def process_order(order_id: str, priority: int) -> Optional[Dict[str, str]]:
    ...
```

### ROS 2 Node Structure
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Logging
Use ROS 2 logger: `self.get_logger().info()`, `.warn()`, `.error()`, `.debug()`

### Error Handling
```python
try:
    result = process_data(data)
except ValueError as e:
    self.get_logger().error(f'Invalid data: {e}')
    return None
```

### Docstrings (PEP 257)
```python
def generate_order(self):
    """Generate a new order with unique ID and publish to /orders topic."""
    ...
```

### Copyright Header (Apache 2.0)
```python
# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
```

### File Organization
```
package_name/
├── package_name/
│   ├── __init__.py
│   ├── node1.py
│   └── node2.py
├── launch/
│   └── launch_file.launch.py
├── test/
│   ├── test_flake8.py
│   ├── test_pep257.py
│   └── test_copyright.py
├── setup.py
├── package.xml
└── README.md
```

### ROS 2 Best Practices
- Use QoS profiles for publishers/subscribers when needed
- Use timers for periodic tasks instead of sleep loops
- Clean up resources in `finally` block
- Keep nodes focused on single responsibility
- Use actions for long-running tasks needing feedback

## Running the System
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch order_system complete_warehouse.launch.py
```

## Package Notes
- `amr_description/scripts/isaaclab_warehouse.py` - IsaacLab only
- `amr_description/worlds/warehouse.world` - Gazebo world
- `amr_description/urdf/amr_robot.urdf` - Robot description
- `cuopt_bridge/` - Mock CuOpt (replace with real API)
