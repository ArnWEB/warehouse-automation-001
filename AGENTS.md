# AGENTS.md - Warehouse Automation Project

## Project Overview

This is a ROS 2 Humble warehouse automation project with the following packages:
- `order_system` - Order generation and listening
- `cuopt_bridge` - Mock NVIDIA CuOpt optimizer
- `orchestrator` - Task execution and robot management
- `amr_description` - Robot URDF, Gazebo world, Isaac Sim scripts
- `warehouse_msgs` - Custom message definitions
- `hello_world_pkgs` - Example packages

## Build Commands

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Rebuild after changes
colcon build --packages-select <package_name> --symlink-install
```

## Lint Commands

```bash
# Run all linters (flake8, pep257, copyright)
colcon test

# Run specific linter on a package
cd <package_name>
ament_flake8 .
ament_pep257 .

# Run single test file
pytest test/test_flake8.py -v
pytest test/test_pep257.py -v
pytest test/test_copyright.py -v
```

## Test Commands

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# Run tests with verbose output
colcon test --event-handlers console_direct+

# Run pytest directly on a test file
cd <package_name>
python3 -m pytest test/test_flake8.py -v

# Run a specific test function
python3 -m pytest test/test_flake8.py::test_flake8 -v
```

## Code Style Guidelines

### General
- Use Python 3
- Follow PEP 8 (enforced by flake8)
- Use shebang `#!/usr/bin/env python3` for executable scripts
- Maximum line length: 100 characters

### Imports
- Standard library first, then third-party, then ROS/ROS 2
- Group in order: stdlib, third-party, ROS messages, ROS nodes
- Use absolute imports (e.g., `from rclpy.node import Node`)
- Sort alphabetically within groups

```python
#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

### Naming Conventions
- **Classes**: PascalCase (e.g., `OrderGenerator`)
- **Functions/methods**: snake_case (e.g., `generate_order`)
- **Variables**: snake_case (e.g., `order_count`)
- **Constants**: UPPER_SNAKE_CASE (e.g., `MAX_RETRIES`)
- **ROS topics/services**: snake_case with slashes (e.g., `/orders`, `/robot_status`)

### Type Annotations
- Use type hints for function parameters and return values
- Use `typing` module for complex types

```python
from typing import List, Dict, Optional

def process_order(order_id: str, priority: int) -> Dict[str, any]:
    ...
```

### ROS 2 Node Structure
- Inherit from `rclpy.node.Node`
- Use `main()` function with `rclpy.init()` and `rclpy.spin()`
- Handle `KeyboardInterrupt` gracefully

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Initialize publishers, subscribers, timers

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
- Use ROS 2 logger: `self.get_logger().info('message')`
- Log levels: debug, info, warn, error, fatal

```python
self.get_logger().info('Starting node')
self.get_logger().warn('Something unexpected')
self.get_logger().error('Failed to process')
```

### Error Handling
- Use try/except for critical sections
- Catch specific exceptions when possible
- Re-raise or handle appropriately

```python
try:
    result = process_data(data)
except ValueError as e:
    self.get_logger().error(f'Invalid data: {e}')
    return None
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

### Docstrings
- Use PEP 257 style (one-line for simple, multi-line for complex)
- Include description, args, returns for functions

```python
def generate_order(self):
    """Generate a new order with unique ID and publish to /orders topic."""
    ...
```

### Copyright Header
Include Apache 2.0 license header in new files:

```python
# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

### ROS 2 Best Practices
- Always use QoS profiles for publishers/subscribers when needed
- Use timers for periodic tasks instead of sleep loops
- Clean up resources in `finally` block
- Use meaningful node and topic names
- Keep nodes focused on single responsibility
- Use actions for long-running tasks that may need feedback

## Running the System

```bash
# Terminal 1: Start Gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch order_system complete_warehouse.launch.py

# Terminal 2: Run Isaac Sim warehouse (optional)
cd ~/IsaacLab
./isaaclab.sh -p /path/to/warehouse-automation/amr_description/scripts/isaaclab_warehouse.py
```

## Package-Specific Notes

- `amr_description/scripts/isaaclab_warehouse.py` - Runs in IsaacLab environment only
- `amr_description/worlds/warehouse.world` - Gazebo world file
- `amr_description/urdf/amr_robot.urdf` - Robot description
- `cuopt_bridge/` - Contains mock CuOpt implementation (replace with real API)
