# ROS2 Node Rules

## BaseNode Pattern (Python)

All nodes inherit from a `BaseNode` that wraps `rclpy.node.Node`.
Infrastructure code stays in the node; domain/application logic stays outside.

```python
# infrastructure/ros2/base_node.py
import rclpy
from rclpy.node import Node

class BaseNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} started")
```

## Publisher Node Template (Python)

```python
from rclpy.node import Node
from std_msgs.msg import Float32

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self._pub = self.create_publisher(Float32, '/my_topic', 10)
        self.create_timer(0.1, self._timer_cb)

    def _timer_cb(self):
        msg = Float32()
        msg.data = 1.0
        self._pub.publish(msg)
```

## Subscriber Node Template (Python)

```python
from rclpy.node import Node
from std_msgs.msg import Float32

class MySubscriberNode(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.create_subscription(Float32, '/my_topic', self._cb, 10)

    def _cb(self, msg: Float32):
        self.get_logger().info(f"Received: {msg.data}")
```

## BaseNode Pattern (C++)

```cpp
#include "rclcpp/rclcpp.hpp"

class BaseNode : public rclcpp::Node {
public:
  explicit BaseNode(const std::string & node_name)
  : rclcpp::Node(node_name) {
    RCLCPP_INFO(get_logger(), "%s started", node_name.c_str());
  }
};
```
