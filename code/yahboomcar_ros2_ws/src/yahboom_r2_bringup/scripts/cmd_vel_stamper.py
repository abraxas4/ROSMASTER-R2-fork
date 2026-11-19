#!/usr/bin/env python3
"""geometry_msgs/Twist (/cmd_vel) -> TwistStamped (ackermann_steering_controller/reference).

Jazzy's steering controllers only accept TwistStamped. Yahboom joystick / Nav2 publish Twist.
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('frame_id', 'base_footprint')
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
