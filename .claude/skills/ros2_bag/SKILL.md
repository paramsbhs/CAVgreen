---
name: ROS2 Bag
description: Record and replay ROS2 bag files programmatically and via CLI
---

# ROS2 Bag

## When to use this skill

- Recording sensor data for offline analysis
- Replaying recorded data for testing
- Programmatic bag recording within a node

## CLI Recording & Replay

```bash
# Record specific topics
ros2 bag record -o my_bag /scan /odom /cmd_vel

# Record all topics
ros2 bag record -o my_bag -a

# Replay
ros2 bag play my_bag

# Replay at half speed
ros2 bag play my_bag --rate 0.5

# Replay looping
ros2 bag play my_bag --loop

# Bag info
ros2 bag info my_bag
```

## Programmatic Recording (rosbag2_py)

```python
import rosbag2_py
from rclpy.serialization import serialize_message

writer = rosbag2_py.SequentialWriter()
storage_options = rosbag2_py.StorageOptions(uri='my_bag', storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
writer.open(storage_options, converter_options)

topic_info = rosbag2_py.TopicMetadata(
    name='/my_topic',
    type='std_msgs/msg/Float32',
    serialization_format='cdr')
writer.create_topic(topic_info)

# Write a message
from std_msgs.msg import Float32
msg = Float32(); msg.data = 1.0
writer.write('/my_topic', serialize_message(msg), 0)
del writer  # closes the bag
```

## Bag Replay for Testing

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri='my_bag', storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/my_topic':
        msg = deserialize_message(data, Float32)
        print(msg.data)
```
