# Testing Rules

## Test Structure

```
test/
├── unit/         # Pure domain/application logic, no ROS2
├── integration/  # ROS2 node tests using rclpy
└── e2e/          # Full stack launch tests
```

## Unit Test Template (pytest)

```python
# test/unit/test_my_entity.py
from domain.entities.my_entity import MyEntity

def test_valid_entity():
    entity = MyEntity(value=1.0)
    assert entity.is_valid()

def test_invalid_entity():
    entity = MyEntity(value=-1.0)
    assert not entity.is_valid()
```

## ROS2 Node Integration Test Template

```python
import pytest
import rclpy
from rclpy.node import Node

@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_starts(ros_context):
    node = Node("test_node")
    assert node.get_name() == "test_node"
    node.destroy_node()
```

## Commands

| Action        | Command                              |
| ------------- | ------------------------------------ |
| Run all tests | `colcon test`                        |
| View results  | `colcon test-result --all`           |
| Run pytest    | `pytest test/unit/`                  |
| Verbose       | `pytest test/unit/ -v`               |
