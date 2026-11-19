#!/usr/bin/env python3
"""
Wheel slip detector for R2 robot.
Compares wheel odometry motion with actual motion from SLAM/scan matching.
Uses TF map->base_footprint or SLAM pose as ground truth (independent of wheel encoders).
Publishes slip status and triggers emergency stop if slipping persists for 3+ seconds.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')
        
        self.declare_parameter('slip_threshold_linear', 0.10)
        self.declare_parameter('slip_threshold_angular', 0.15)
        self.declare_parameter('slip_duration_limit', 3.0)
        self.declare_parameter('check_rate', 10.0)
        self.declare_parameter('wheel_odom_topic', '/ackermann_steering_controller/odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.slip_threshold_linear = self.get_parameter('slip_threshold_linear').value
        self.slip_threshold_angular = self.get_parameter('slip_threshold_angular').value
        self.slip_duration_limit = self.get_parameter('slip_duration_limit').value
        self.check_rate = self.get_parameter('check_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.wheel_odom = None
        self.last_map_pose = None
        self.last_map_time = None
        self.is_slipping = False
        self.slip_start_time = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        
        self.slip_pub = self.create_publisher(Bool, 'slip_detected', 10)
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(1.0 / self.check_rate, self.check_slip)
        
        self.get_logger().info(
            f'Slip detector initialized. Comparing wheel odom vs TF {self.map_frame}->{self.base_frame}'
        )
        
    def wheel_odom_callback(self, msg):
        self.wheel_odom = msg
        
    def check_slip(self):
        if self.wheel_odom is None:
            return
            
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(
                f'TF lookup failed: {e}. Slip detection inactive.',
                throttle_duration_sec=5.0
            )
            return
            
        current_time = self.get_clock().now()
        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        
        quat = transform.transform.rotation
        current_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        if self.last_map_pose is not None:
            dt = (current_time - self.last_map_time).nanoseconds / 1e9
            if dt > 0.01 and dt < 1.0:
                dx = current_x - self.last_map_pose[0]
                dy = current_y - self.last_map_pose[1]
                dyaw = current_yaw - self.last_map_pose[2]
                
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                
                actual_vx = math.sqrt(dx*dx + dy*dy) / dt
                actual_wz = abs(dyaw) / dt
                
                wheel_vx = abs(self.wheel_odom.twist.twist.linear.x)
                wheel_wz = abs(self.wheel_odom.twist.twist.angular.z)
                
                if wheel_vx < 0.01 and wheel_wz < 0.01:
                    self.reset_slip_state()
                else:
                    linear_diff = abs(wheel_vx - actual_vx)
                    angular_diff = abs(wheel_wz - actual_wz)
                    
                    is_slipping_now = (
                        linear_diff > self.slip_threshold_linear or 
                        angular_diff > self.slip_threshold_angular
                    )
                    
                    if is_slipping_now:
                        if not self.is_slipping:
                            self.is_slipping = True
                            self.slip_start_time = current_time
                            self.get_logger().warn(
                                f'Slip detected! Wheel vx: {wheel_vx:.3f}, Actual vx: {actual_vx:.3f}, '
                                f'Wheel wz: {wheel_wz:.3f}, Actual wz: {actual_wz:.3f}'
                            )
                        else:
                            elapsed = (current_time - self.slip_start_time).nanoseconds / 1e9
                            if elapsed >= self.slip_duration_limit:
                                self.get_logger().error(
                                    f'Slip persisted for {elapsed:.1f}s. Emergency stop!'
                                )
                                self.emergency_stop()
                    else:
                        self.reset_slip_state()
                        
        self.last_map_pose = (current_x, current_y, current_yaw)
        self.last_map_time = current_time
            
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
