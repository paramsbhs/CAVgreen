# ROS2 Common Commands

## Build System (colcon)

| Action          | Command                                    |
| --------------- | ------------------------------------------ |
| Build Workspace | `colcon build --symlink-install`           |
| Build Package   | `colcon build --packages-select <pkg>`     |
| Test            | `colcon test`                              |
| Test Output     | `colcon test-result --all`                 |
| Clean           | `rm -rf build/ install/ log/`              |

## Environment

| Action          | Command                                    |
| --------------- | ------------------------------------------ |
| Source workspace| `source install/setup.bash`                |
| Source ROS2     | `source /opt/ros/humble/setup.bash`        |

## Introspection

| Command                        | Description                        |
| ------------------------------ | ---------------------------------- |
| `ros2 node list`               | List all running nodes             |
| `ros2 node info <node>`        | Show node details                  |
| `ros2 topic list`              | List all active topics             |
| `ros2 topic echo <topic>`      | Print messages on a topic          |
| `ros2 topic hz <topic>`        | Show publish rate                  |
| `ros2 service list`            | List all active services           |
| `ros2 service call <svc> <type> <args>` | Call a service            |
| `ros2 param list`              | List all parameters                |
| `ros2 param get <node> <param>`| Get a parameter value              |
| `ros2 param set <node> <param> <val>` | Set a parameter value       |
| `ros2 action list`             | List all active action servers     |

## Launch

| Action          | Command                                          |
| --------------- | ------------------------------------------------ |
| Run launch file | `ros2 launch <pkg> <launch_file>.launch.py`      |
| Run node        | `ros2 run <pkg> <executable>`                    |

## Bag Files

| Action          | Command                                          |
| --------------- | ------------------------------------------------ |
| Record          | `ros2 bag record -o <name> <topic1> <topic2>`    |
| Play            | `ros2 bag play <bag_dir>`                        |
| Info            | `ros2 bag info <bag_dir>`                        |
