---
name: ROS2 Messaging
description: Pub/Sub patterns, thread-safe buffers, message synchronization
---

# ROS2 Messaging

## When to use this skill

- Setting up publishers and subscribers with correct QoS
- Synchronizing multiple topic streams
- Thread-safe data sharing between callbacks and timers

## Publisher with QoS

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

class ScanPublisher(Node):
    def __init__(self):
        super().__init__('scan_publisher')
        self._pub = self.create_publisher(LaserScan, '/scan', SENSOR_QOS)
```

## Thread-Safe Buffer

```python
import threading
from collections import deque

class MessageBuffer:
    def __init__(self, maxlen: int = 100):
        self._buf: deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, msg) -> None:
        with self._lock:
            self._buf.append(msg)

    def pop(self):
        with self._lock:
            return self._buf.popleft() if self._buf else None
```

## Message Filter Synchronization (ApproximateTimeSynchronizer)

```python
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        img_sub = message_filters.Subscriber(self, Image, '/image')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/camera_info')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, info_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._cb)

    def _cb(self, img, info):
        self.get_logger().info("Synced callback")
```
