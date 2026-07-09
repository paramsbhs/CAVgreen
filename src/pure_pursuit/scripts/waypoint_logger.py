#!/usr/bin/env python3
"""ROS 2 port of the f1tenth waypoint logger.

Subscribes to a localization topic (relative name "odom": ground-truth odom in
sim, particle-filter odom on the car) and appends [x, y, yaw, speed] rows to a
CSV whenever the car has moved at least `min_spacing` metres.

Usage:
  ros2 run pure_pursuit waypoint_logger.py --ros-args \
      -r odom:=/ego_racecar/odom -p output_csv:=/tmp/waypoints.csv
Drive the car around the track (keyboard teleop or gap_follow), then Ctrl-C.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.declare_parameter('output_csv', '/tmp/waypoints.csv')
        self.declare_parameter('min_spacing', 0.10)
        self.output_csv = self.get_parameter('output_csv').value
        self.min_spacing = self.get_parameter('min_spacing').value
        self.file = open(self.output_csv, 'w')
        self.file.write('x,y,yaw,speed\n')
        self.count = 0
        self.last_xy = None
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.get_logger().info(f'Logging waypoints to {self.output_csv}')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None:
            if math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < self.min_spacing:
                return
        self.last_xy = (x, y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x
        self.file.write(f'{x:.4f},{y:.4f},{yaw:.4f},{v:.3f}\n')
        self.file.flush()
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f'{self.count} waypoints logged')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.get_logger().info(f'Saved {node.count} waypoints to {node.output_csv}')
        node.destroy_node()


if __name__ == '__main__':
    main()
