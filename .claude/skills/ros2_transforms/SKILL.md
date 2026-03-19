---
name: ROS2 Transforms (TF2)
description: Publish and lookup TF2 transforms following Clean Architecture
---

# ROS2 Transforms (TF2)

## When to use this skill

- Publishing static or dynamic transforms
- Looking up transform between frames
- Integrating TF2 in a Clean Architecture node

## Static Transform Publisher (Python)

```python
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import math

class StaticTFNode(Node):
    def __init__(self):
        super().__init__('static_tf_node')
        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transform()

    def _publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = 0.2
        t.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(t)
```

## Dynamic Transform Lookup

```python
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def get_transform(self, target: str, source: str):
        try:
            return self._buffer.lookup_transform(
                target, source, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(str(e))
            return None
```

## Clean Architecture: TF Port

```python
# application/interfaces/transform_provider.py
from abc import ABC, abstractmethod

class TransformProvider(ABC):
    @abstractmethod
    def get_transform(self, target_frame: str, source_frame: str): ...
```
