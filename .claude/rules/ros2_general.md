# ROS2 General Rules

## Package Naming

- Use `snake_case` for all package names.
- Prefix with project name, e.g., `cavgreen_perception`, `cavgreen_control`.

## File Structure

```
<package>/
├── package.xml
├── CMakeLists.txt  (C++) or setup.py (Python)
├── <package>/
│   ├── __init__.py
│   ├── domain/
│   ├── application/
│   └── infrastructure/
├── launch/
├── config/
├── test/
└── resource/
```

## CMakeLists.txt Minimal Template (C++)

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## package.xml Minimal Template

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>Description here</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  <depend>rclcpp</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```
