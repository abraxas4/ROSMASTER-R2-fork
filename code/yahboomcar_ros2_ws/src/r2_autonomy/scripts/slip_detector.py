#!/usr/bin/env python3
"""
Wheel slip detector for R2 robot.
Compares wheel-reported motion with scan-matched motion from slam_toolbox (/pose).
No TF listener: /tf (~75 Hz) in rclpy cost ~40% CPU at rest.
- Wheel motion: integrated from odom twist (default /odom, EKF 30 Hz).
- Actual motion: displacement between consecutive /pose messages (map frame,
  published only after scan match, so it is independent of wheel encoders).
Publishes slip status; emergency stop if slipping persists for slip_duration_limit.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')
        self.declare_parameter('slip_threshold_linear', 0.10)
        self.declare_parameter('slip_threshold_angular', 0.15)
        self.declare_parameter('slip_duration_limit', 3.0)
        self.declare_parameter('check_rate', 10.0)  # kept for compatibility (unused)
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('max_pose_gap', 5.0)

        self.thr_lin = self.get_parameter('slip_threshold_linear').value
        self.thr_ang = self.get_parameter('slip_threshold_angular').value
        self.dur_limit = self.get_parameter('slip_duration_limit').value
        self.max_gap = self.get_parameter('max_pose_gap').value

        self.last_odom_t = None
        self.wheel_dist = 0.0   # integrated |v| since last pose
        self.wheel_ang = 0.0    # integrated |w| since last pose
        self.last_pose = None   # (x, y, yaw, t)
        self.is_slipping = False
        self.slip_start = None

        self.create_subscription(Odometry, self.get_parameter('wheel_odom_topic').value,
                                 self.odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').value,
                                 self.pose_cb, 10)
        self.slip_pub = self.create_publisher(Bool, 'slip_detected', 10)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Slip detector: wheel odom vs slam /pose (no TF listener)')

    def odom_cb(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        if self.last_odom_t is not None:
            dt = t - self.last_odom_t
            if 0.0 < dt < 0.5:
                self.wheel_dist += abs(msg.twist.twist.linear.x) * dt
                self.wheel_ang += abs(msg.twist.twist.angular.z) * dt
        self.last_odom_t = t

    def pose_cb(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if self.last_pose is not None:
            dt = t - self.last_pose[3]
            if 0.05 < dt < self.max_gap:
                actual_v = math.hypot(x - self.last_pose[0], y - self.last_pose[1]) / dt
                dyaw = math.atan2(math.sin(yaw - self.last_pose[2]), math.cos(yaw - self.last_pose[2]))
                actual_w = abs(dyaw) / dt
                wheel_v = self.wheel_dist / dt
                wheel_w = self.wheel_ang / dt
                if wheel_v < 0.01 and wheel_w < 0.01:
                    self.reset()
                elif abs(wheel_v - actual_v) > self.thr_lin or abs(wheel_w - actual_w) > self.thr_ang:
                    now = self.get_clock().now()
                    if not self.is_slipping:
                        self.is_slipping = True
                        self.slip_start = now
                        self.get_logger().warn(
                            f'Slip detected! wheel v {wheel_v:.3f} vs actual {actual_v:.3f}, '
                            f'wheel w {wheel_w:.3f} vs actual {actual_w:.3f}')
                    elif (now - self.slip_start).nanoseconds / 1e9 >= self.dur_limit:
                        self.get_logger().error('Slip persisted. Emergency stop!')
                        self.emergency_stop()
                else:
                    self.reset()
            else:
                self.reset()
        self.last_pose = (x, y, yaw, t)
        self.wheel_dist = 0.0
        self.wheel_ang = 0.0
        self.slip_pub.publish(Bool(data=self.is_slipping))

    def reset(self):
        if self.is_slipping:
            self.get_logger().info('Slip resolved')
        self.is_slipping = False
        self.slip_start = None

    def emergency_stop(self):
        for _ in range(5):
            self.stop_pub.publish(Twist())


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
