#!/usr/bin/env python3
"""Publish /odom from TF odom->base_footprint (Cartographer pose, not wheel EKF)."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(t: TransformStamped) -> float:
    q = t.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TfToOdom(Node):
    def __init__(self) -> None:
        super().__init__('tf_to_odom')
        self.parent = self.declare_parameter('parent_frame', 'odom').value
        self.child = self.declare_parameter('child_frame', 'base_footprint').value
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.prev: TransformStamped | None = None
        self.create_timer(0.05, self._tick)

    def _tick(self) -> None:
        try:
            tf = self.buf.lookup_transform(self.parent, self.child, rclpy.time.Time())
        except TransformException:
            return
        msg = Odometry()
        msg.header = tf.header
        msg.header.frame_id = self.parent
        msg.child_frame_id = self.child
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        if self.prev is not None:
            dt = (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9) - (
                self.prev.header.stamp.sec + self.prev.header.stamp.nanosec * 1e-9
            )
            if dt > 1e-4:
                dx = tf.transform.translation.x - self.prev.transform.translation.x
                dy = tf.transform.translation.y - self.prev.transform.translation.y
                dyaw = yaw_of(tf) - yaw_of(self.prev)
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                msg.twist.twist.linear.x = math.hypot(dx, dy) / dt
                msg.twist.twist.angular.z = dyaw / dt
        self.prev = tf
        self.pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = TfToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
