---
name: ROS2 Lifecycle Nodes
description: Create managed lifecycle nodes with clean state transitions
---

# ROS2 Lifecycle Nodes

## When to use this skill

- Node needs deterministic startup/shutdown
- Resource allocation must be tied to activation state
- Coordinating multi-node systems via lifecycle manager

## Lifecycle Node Template (Python)

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

    def on_configure(self, state):
        self._pub = self.create_lifecycle_publisher(Float32, '/output', 10)
        self.get_logger().info("Configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._pub.on_activate(state)
        self._timer = self.create_timer(0.1, self._timer_cb)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._pub.on_deactivate(state)
        self._timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.destroy_publisher(self._pub)
        return TransitionCallbackReturn.SUCCESS

    def _timer_cb(self):
        msg = Float32(); msg.data = 1.0
        self._pub.publish(msg)
```

## State Transitions

```
Unconfigured → configure → Inactive → activate → Active
Active → deactivate → Inactive → cleanup → Unconfigured
Any → shutdown → Finalized
```

## CLI Commands

```bash
ros2 lifecycle set /my_lifecycle_node configure
ros2 lifecycle set /my_lifecycle_node activate
ros2 lifecycle set /my_lifecycle_node deactivate
ros2 lifecycle get /my_lifecycle_node
```
