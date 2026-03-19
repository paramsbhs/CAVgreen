---
name: ROS2 Service & Action
description: Create ROS2 service servers/clients and action servers/clients
---

# ROS2 Service & Action

## When to use this skill

- Implementing request/response communication (services)
- Implementing long-running tasks with feedback (actions)

## Service Server (Python)

```python
from std_srvs.srv import SetBool
from rclpy.node import Node

class MyServiceNode(Node):
    def __init__(self):
        super().__init__('my_service_node')
        self.create_service(SetBool, '/my_service', self._handle)

    def _handle(self, request, response):
        self.get_logger().info(f"Request: {request.data}")
        response.success = True
        response.message = "OK"
        return response
```

## Service Client (Python)

```python
from std_srvs.srv import SetBool
from rclpy.node import Node

class MyClientNode(Node):
    def __init__(self):
        super().__init__('my_client_node')
        self._client = self.create_client(SetBool, '/my_service')
        self._client.wait_for_service()

    def call(self, data: bool):
        req = SetBool.Request()
        req.data = data
        future = self._client.call_async(req)
        return future
```

## Action Server (Python)

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from my_msgs.action import MyAction

class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        self._server = ActionServer(self, MyAction, '/my_action', self._execute)

    async def _execute(self, goal_handle):
        feedback = MyAction.Feedback()
        for i in range(10):
            feedback.feedback_value = float(i)
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = MyAction.Result()
        result.result_value = 10.0
        return result
```
