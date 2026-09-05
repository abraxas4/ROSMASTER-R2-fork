#!/usr/bin/env python3
"""Seed AMCL at map origin. Start the robot where Cartographer mapping started."""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser(description='Publish Nav2 /initialpose')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('publish_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.pose.position.x = args.x
    pose.pose.pose.position.y = args.y
    pose.pose.pose.orientation.z = float(__import__('math').sin(args.yaw / 2.0))
    pose.pose.pose.orientation.w = float(__import__('math').cos(args.yaw / 2.0))
    pose.pose.covariance[0] = 0.25
    pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.07

    for _ in range(8):
        pose.header.stamp = node.get_clock().now().to_msg()
        pub.publish(pose)
        rclpy.spin_once(node, timeout_sec=0.2)

    node.get_logger().info(
        f'Initial pose published: ({args.x:.2f}, {args.y:.2f}, yaw={args.yaw:.2f})'
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
