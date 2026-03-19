# Robot-Specific Rules

## TF2 Frame Naming

- World frame: `map`
- Robot base: `base_link`
- Sensor frames: `<sensor>_link`, e.g., `lidar_link`, `camera_link`

## URDF/Xacro Template

```xml
<?xml version="1.0"?>
<robot name="cavgreen" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.2"/></geometry>
    </visual>
  </link>
</robot>
```

## Nav2 Integration

- Use `nav2_bringup` for standard navigation stack.
- Override parameters via YAML in `config/nav2_params.yaml`.
- Always specify `use_sim_time: true` in simulation.

## Sensor Integration

- LaserScan: `sensor_msgs/msg/LaserScan`
- PointCloud: `sensor_msgs/msg/PointCloud2`
- Image: `sensor_msgs/msg/Image`
- IMU: `sensor_msgs/msg/Imu`
- Odometry: `nav_msgs/msg/Odometry`
