#!/usr/bin/env python3
"""
Wheel slip detector for R2 robot.
Compares wheel odometry motion with actual motion from localization/scan matching.
Publishes slip status and triggers emergency stop if slipping persists for 3+ seconds.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool
import tf2_ros
import math
from rclpy.duration import Duration


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')
        
        self.declare_parameter('slip_threshold_linear', 0.10)
        self.declare_parameter('slip_threshold_angular', 0.15)
        self.declare_parameter('slip_duration_limit', 3.0)
        self.declare_parameter('check_rate', 10.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_odom_topic', '/ackermann_steering_controller/odometry')
        
        self.slip_threshold_linear = self.get_parameter('slip_threshold_linear').value
        self.slip_threshold_angular = self.get_parameter('slip_threshold_angular').value
        self.slip_duration_limit = self.get_parameter('slip_duration_limit').value
        self.check_rate = self.get_parameter('check_rate').value
        
        self.wheel_odom = None
        self.fused_odom = None
        self.is_slipping = False
        self.slip_start_time = None
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('wheel_odom_topic').value,
            self.wheel_odom_callback,
            qos
        )
        
        self.fused_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.fused_odom_callback,
            qos
        )
        
        self.slip_pub = self.create_publisher(Bool, 'slip_detected', 10)
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(1.0 / self.check_rate, self.check_slip)
        
        self.get_logger().info('Slip detector initialized')
        
    def wheel_odom_callback(self, msg):
        self.wheel_odom = msg
        
    def fused_odom_callback(self, msg):
        self.fused_odom = msg
        
    def check_slip(self):
        if self.wheel_odom is None or self.fused_odom is None:
            return
            
        wheel_vx = self.wheel_odom.twist.twist.linear.x
        wheel_wz = self.wheel_odom.twist.twist.angular.z
        
        fused_vx = self.fused_odom.twist.twist.linear.x
        fused_wz = self.fused_odom.twist.twist.angular.z
        
        if abs(wheel_vx) < 0.01 and abs(wheel_wz) < 0.01:
            self.reset_slip_state()
            return
            
        linear_diff = abs(wheel_vx - fused_vx)
        angular_diff = abs(wheel_wz - fused_wz)
        
        is_slipping_now = (
            linear_diff > self.slip_threshold_linear or 
            angular_diff > self.slip_threshold_angular
        )
        
        if is_slipping_now:
            if not self.is_slipping:
                self.is_slipping = True
                self.slip_start_time = self.get_clock().now()
                self.get_logger().warn(
                    f'Slip detected! Linear diff: {linear_diff:.3f}, '
                    f'Angular diff: {angular_diff:.3f}'
                )
            else:
                elapsed = (self.get_clock().now() - self.slip_start_time).nanoseconds / 1e9
                if elapsed >= self.slip_duration_limit:
                    self.get_logger().error(
                        f'Slip persisted for {elapsed:.1f}s. Emergency stop!'
                    )
                    self.emergency_stop()
        else:
            self.reset_slip_state()
            
        slip_msg = Bool()
        slip_msg.data = self.is_slipping
        self.slip_pub.publish(slip_msg)
        
    def reset_slip_state(self):
        if self.is_slipping:
            self.get_logger().info('Slip resolved')
        self.is_slipping = False
        self.slip_start_time = None
        
    def emergency_stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(5):
            self.emergency_stop_pub.publish(stop_msg)
            

def main(args=None):
    rclpy.init(args=args)
    node = SlipDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
