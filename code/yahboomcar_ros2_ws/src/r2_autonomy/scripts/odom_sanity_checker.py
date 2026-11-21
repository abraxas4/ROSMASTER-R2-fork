#!/usr/bin/env python3
"""
Odometry sanity checker for R2 robot.
Detects and clamps unrealistic odom jumps (e.g. to 649m) that can crash Cartographer.
Publishes sanitized odometry on /odom_safe.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import math


class OdomSanityChecker(Node):
    def __init__(self):
        super().__init__('odom_sanity_checker')
        
        self.declare_parameter('max_jump_distance', 5.0)
        self.declare_parameter('max_position', 100.0)
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_safe')
        
        self.max_jump = self.get_parameter('max_jump_distance').value
        self.max_pos = self.get_parameter('max_position').value
        
        self.last_odom = None
        self.correction_offset_x = 0.0
        self.correction_offset_y = 0.0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('input_topic').value,
            self.odom_callback,
            qos
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('output_topic').value,
            qos
        )
        
        self.get_logger().info(
            f'Odom sanity checker initialized. Max jump: {self.max_jump}m, '
            f'Max position: {self.max_pos}m'
        )
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if abs(x) > self.max_pos or abs(y) > self.max_pos:
            self.get_logger().error(
                f'Odom position out of bounds: ({x:.2f}, {y:.2f}). '
                f'Applying correction offset.'
            )
            self.correction_offset_x = -x
            self.correction_offset_y = -y
            
        if self.last_odom is not None:
            last_x = self.last_odom.pose.pose.position.x
            last_y = self.last_odom.pose.pose.position.y
            
            dx = x - last_x
            dy = y - last_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > self.max_jump:
                self.get_logger().error(
                    f'Odom jump detected: {distance:.2f}m from '
                    f'({last_x:.2f}, {last_y:.2f}) to ({x:.2f}, {y:.2f}). '
                    f'Applying correction.'
                )
                self.correction_offset_x += last_x - x
                self.correction_offset_y += last_y - y
                
        corrected_msg = Odometry()
        corrected_msg.header = msg.header
        corrected_msg.child_frame_id = msg.child_frame_id
        
        corrected_msg.pose.pose = msg.pose.pose
        corrected_msg.pose.pose.position.x = x + self.correction_offset_x
        corrected_msg.pose.pose.position.y = y + self.correction_offset_y
        corrected_msg.pose.covariance = msg.pose.covariance
        
        corrected_msg.twist = msg.twist
        
        self.odom_pub.publish(corrected_msg)
        self.last_odom = corrected_msg


def main(args=None):
    rclpy.init(args=args)
    node = OdomSanityChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
