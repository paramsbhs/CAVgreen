# ROS2 Communication Rules

## Topic Naming Convention

- Use namespaced, descriptive names: `/<robot>/<subsystem>/<signal>`
- Example: `/cavgreen/sensors/lidar_scan`

## QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)
```

## Custom Message Definition

```
# msg/MyMsg.msg
float32 value
string label
builtin_interfaces/Time stamp
```

## Custom Service Definition

```
# srv/MyService.srv
float32 input
---
float32 output
bool success
```

## Custom Action Definition

```
# action/MyAction.action
float32 goal_value
---
float32 result_value
---
float32 feedback_value
```
