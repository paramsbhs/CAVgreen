---
name: ROS2 Node Creation
description: Create Clean Architecture compliant ROS2 nodes in Python or C++
---

# ROS2 Node Creation

Creates ROS2 nodes following Clean Architecture: domain logic is isolated from ROS2 infrastructure.

## When to use this skill

- Creating a new ROS2 node from scratch
- Refactoring an existing node to follow Clean Architecture
- Adding publishers, subscribers, timers, or parameters to a node

## Python Node Template

```python
# infrastructure/ros2/my_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from application.use_cases.my_use_case import MyUseCase
from infrastructure.repositories.my_repository_impl import MyRepositoryImpl


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        repo = MyRepositoryImpl()
        self._use_case = MyUseCase(repo)
        self._pub = self.create_publisher(Float32, '/output', 10)
        self.create_subscription(Float32, '/input', self._cb, 10)
        self.get_logger().info("MyNode started")

    def _cb(self, msg: Float32):
        result = self._use_case.execute(msg.data)
        out = Float32()
        out.data = result.value
        self._pub.publish(out)


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## C++ Node Template

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : rclcpp::Node("my_node") {
    pub_ = create_publisher<std_msgs::msg::Float32>("/output", 10);
    sub_ = create_subscription<std_msgs::msg::Float32>(
      "/input", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) { callback(msg); });
    RCLCPP_INFO(get_logger(), "MyNode started");
  }

private:
  void callback(std_msgs::msg::Float32::SharedPtr msg) {
    auto out = std_msgs::msg::Float32();
    out.data = msg->data;
    pub_->publish(out);
  }
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
```
