---
name: ROS2 Diagnostics
description: Add health monitoring using diagnostic_updater
---

# ROS2 Diagnostics

## When to use this skill

- Adding health/status monitoring to a node
- Publishing to `/diagnostics` topic
- Integrating with `rqt_robot_monitor`

## diagnostic_updater Template (Python)

```python
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

class MonitoredNode(Node):
    def __init__(self):
        super().__init__('monitored_node')
        self._updater = Updater(self)
        self._updater.setHardwareID("cavgreen_robot")
        self._updater.add("Sensor Status", self._check_sensor)
        self._ok = True

    def _check_sensor(self, stat: DiagnosticStatusWrapper):
        if self._ok:
            stat.summary(DiagnosticStatusWrapper.OK, "Sensor nominal")
        else:
            stat.summary(DiagnosticStatusWrapper.ERROR, "Sensor failure")
        stat.add("last_reading", str(self._last_value))
        return stat
```

## Domain Health Entity

```python
# domain/entities/health_status.py
from dataclasses import dataclass
from enum import Enum

class HealthLevel(Enum):
    OK = "ok"
    WARN = "warn"
    ERROR = "error"

@dataclass
class HealthStatus:
    level: HealthLevel
    message: str

    def is_ok(self) -> bool:
        return self.level == HealthLevel.OK
```
