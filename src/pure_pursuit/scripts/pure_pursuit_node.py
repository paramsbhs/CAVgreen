#!/usr/bin/env python3
"""Pure pursuit waypoint tracker (F1TENTH Lab 5).

Subscribes to a localization topic (relative name "odom": ground-truth odom in
sim, particle-filter odom on the car) and publishes AckermannDriveStamped on
the relative "drive" topic; launch files remap both per environment.

Waypoints come from a CSV with header x,y,yaw,speed (closed loop implied).
"""
import csv
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

WHEELBASE = 0.33  # m


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('waypoint_csv', '')
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('lookahead_gain', 0.45)
        self.declare_parameter('lookahead_min', 1.0)
        self.declare_parameter('lookahead_max', 2.5)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('velocity_scale', 1.0)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('fallback_speed', 2.0)
        self.declare_parameter('publish_markers', True)

        p = self.get_parameter
        self.L_fixed = p('lookahead_distance').value
        self.L_gain = p('lookahead_gain').value
        self.L_min = p('lookahead_min').value
        self.L_max = p('lookahead_max').value
        self.max_steer = p('max_steering_angle').value
        self.v_scale = p('velocity_scale').value
        self.v_max = p('max_speed').value
        self.v_fallback = p('fallback_speed').value
        self.markers_on = p('publish_markers').value

        csv_path = p('waypoint_csv').value
        self.wp = self.load_waypoints(csv_path)
        self.n_wp = len(self.wp)
        self.get_logger().info(f'Loaded {self.n_wp} waypoints from {csv_path}')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_subscription(Odometry, 'odom', self.pose_callback, 10)

        if self.markers_on:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path_pub = self.create_publisher(MarkerArray, 'waypoints_viz', latched)
            self.target_pub = self.create_publisher(Marker, 'target_viz', 10)
            self.publish_path_markers()

    def load_waypoints(self, path):
        rows = []
        with open(path) as f:
            for r in csv.DictReader(f):
                rows.append((float(r['x']), float(r['y']), float(r.get('speed', 0) or 0)))
        if len(rows) < 2:
            raise RuntimeError(f'waypoint CSV {path} has fewer than 2 points')
        return np.array(rows)  # columns: x, y, speed

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v_now = msg.twist.twist.linear.x

        # adaptive lookahead
        if self.L_gain > 0.0:
            L = min(max(self.L_gain * abs(v_now), self.L_min), self.L_max)
        else:
            L = self.L_fixed

        # Nearest waypoint, then walk forward (wrapping) to the first one at
        # distance >= L. Walking forward from the nearest point -- never a
        # global argmin over |d - L| -- keeps the target from snapping to a
        # point behind the car or across the track.
        d2 = (self.wp[:, 0] - x) ** 2 + (self.wp[:, 1] - y) ** 2
        idx = int(np.argmin(d2))
        target = idx
        for step in range(self.n_wp):
            j = (idx + step) % self.n_wp
            if math.hypot(self.wp[j, 0] - x, self.wp[j, 1] - y) >= L:
                target = j
                break

        # Transform the goal into the vehicle frame; if it lands behind the
        # rear axle (possible right after a localization jump), walk further
        # forward along the path until it is in front.
        x_vf = y_vf = 0.0
        gx = gy = 0.0
        for _ in range(self.n_wp):
            gx, gy = self.wp[target, 0], self.wp[target, 1]
            dx, dy = gx - x, gy - y
            x_vf = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_vf = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if x_vf > 0.0:
                break
            target = (target + 1) % self.n_wp

        # pure pursuit law: curvature of the arc through the goal point
        L_actual = math.hypot(x_vf, y_vf)
        if L_actual < 1e-3:
            return
        gamma = 2.0 * y_vf / (L_actual ** 2)
        steering = math.atan(WHEELBASE * gamma)
        steering = max(-self.max_steer, min(self.max_steer, steering))

        v_csv = self.wp[target, 2]
        speed = self.v_scale * v_csv if v_csv > 0.05 else self.v_fallback
        speed = min(speed, self.v_max)

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.steering_angle = steering
        out.drive.speed = speed
        self.drive_pub.publish(out)

        if self.markers_on:
            self.publish_target_marker(gx, gy)

    def publish_path_markers(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'waypoints'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.g = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(X), y=float(Y), z=0.05) for X, Y, _ in self.wp]
        m.points.append(m.points[0])  # close the loop
        ma = MarkerArray()
        ma.markers.append(m)
        self.path_pub.publish(ma)

    def publish_target_marker(self, gx, gy):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'target'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(gx)
        m.pose.position.y = float(gy)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r = 1.0
        m.color.a = 1.0
        self.target_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
