# ROS2 Clean Architecture Project

This project is set up with a comprehensive set of **Claude Skills** designed to
facilitate ROS2 development following **Clean Architecture** principles.

## Available Skills

| Skill Name              | Description                               |
| ----------------------- | ----------------------------------------- |
| **ros2_node_creation**  | Create Clean Architecture compliant Nodes |
| **ros2_launch_config**  | Modular Launch files                      |
| **ros2_service_action** | Services and Actions                      |
| **ros2_messaging**      | Pub/Sub Patterns                          |
| **ros2_testing**        | Testing Strategy                          |
| **ros2_lifecycle**      | Managed Nodes                             |
| **ros2_transforms**     | TF2 Management                            |
| **ros2_diagnostics**    | Health Monitoring                         |
| **ros2_bag**            | Data Recording                            |

## Project Structure

- **src/domain/**: Pure business logic. No ROS2 dependencies.
- **src/application/**: Application services and interfaces.
- **src/infrastructure/**: ROS2-specific implementations.

## Common Commands

- **Build**: `colcon build --symlink-install`
- **Test**: `colcon test`
- **Source**: `source install/setup.bash`
