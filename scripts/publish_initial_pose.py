#!/usr/bin/env python3
"""Seed AMCL from the current map->base_footprint TF (RTAB-Map localization)."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def main() -> int:
    rclpy.init()
    node = Node('publish_initial_pose')
    buf = Buffer()
    listener = TransformListener(buf, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    pose = None
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            tf = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            continue
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose.position.x = tf.transform.translation.x
        pose.pose.pose.position.y = tf.transform.translation.y
        pose.pose.pose.position.z = tf.transform.translation.z
        pose.pose.pose.orientation = tf.transform.rotation
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.07
        break

    if pose is None:
        node.get_logger().error('Could not read map->base_footprint TF for initial pose')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    for _ in range(5):
        pub.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(
        'Initial pose published: '
        f'({pose.pose.pose.position.x:.2f}, {pose.pose.pose.position.y:.2f})'
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())