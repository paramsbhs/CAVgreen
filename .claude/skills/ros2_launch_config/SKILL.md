---
name: ROS2 Launch Config
description: Create modular, parameterized ROS2 launch files
---

# ROS2 Launch Config

Creates composable, parameter-driven launch files using Python launch API.

## When to use this skill

- Creating a new launch file for a package
- Composing multiple nodes into one launch
- Loading parameters from YAML config files

## Single Node Launch Template

```python
# launch/my_node.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_package'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[config],
            output='screen',
        )
    ])
```

## Multi-Node Launch Template

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_package')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'sensor.launch.py'))),
        Node(
            package='my_package',
            executable='controller_node',
            output='screen',
        ),
    ])
```

## params.yaml Template

```yaml
my_node:
  ros__parameters:
    speed: 1.0
    frame_id: "base_link"
    debug: false
```
