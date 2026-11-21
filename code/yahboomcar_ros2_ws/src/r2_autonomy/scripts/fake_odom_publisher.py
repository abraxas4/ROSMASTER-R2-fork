#!/usr/bin/env python3
"""Publish fake odometry for testing"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.wheel_pub = self.create_publisher(
            Odometry,
            '/ackermann_steering_controller/odometry',
            10
        )
        self.fused_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.publish_odom)
        self.get_logger().info('Fake odom publisher started')
        
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.1
        
        self.wheel_pub.publish(msg)
        self.fused_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
