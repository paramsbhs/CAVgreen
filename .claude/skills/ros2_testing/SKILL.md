---
name: ROS2 Testing
description: Unit, integration, and E2E test patterns for ROS2 projects
---

# ROS2 Testing

## When to use this skill

- Writing unit tests for domain/application logic
- Writing integration tests for ROS2 nodes
- Setting up launch-based E2E tests

## Unit Test (pytest, no ROS2)

```python
# test/unit/test_entity.py
from domain.entities.my_entity import MyEntity

def test_valid():
    assert MyEntity(1.0).is_valid()

def test_invalid():
    assert not MyEntity(-1.0).is_valid()
```

## ROS2 Node Integration Test

```python
# test/integration/test_my_node.py
import pytest
import rclpy
from std_msgs.msg import Float32
from my_package.infrastructure.ros2.my_node import MyNode

@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_publishes(ros):
    received = []
    node = rclpy.create_node('test_helper')
    node.create_subscription(Float32, '/output', lambda m: received.append(m), 10)

    dut = MyNode()
    pub = node.create_publisher(Float32, '/input', 10)
    msg = Float32(); msg.data = 3.0
    pub.publish(msg)

    import time; time.sleep(0.1)
    rclpy.spin_once(dut, timeout_sec=0.1)
    rclpy.spin_once(node, timeout_sec=0.1)

    assert len(received) > 0
    dut.destroy_node()
    node.destroy_node()
```

## colcon test Commands

```bash
colcon test --packages-select my_package
colcon test-result --all --verbose
pytest test/unit/ -v
```
