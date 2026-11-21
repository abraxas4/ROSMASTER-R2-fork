#!/usr/bin/env python3
"""Log command vs MCU-reported vs EKF velocities at a fixed rate.

Records CSV under ~/r2_logs for later measurement. Does not publish
/cmd_vel and does not send navigation goals.
"""
import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _qos_keep_last(depth=10, reliable=True):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class OdomCompareLogger(Node):
    def __init__(self):
        super().__init__('odom_compare_logger')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('reported_odom_topic', '/ackermann_steering_controller/odometry')
        self.declare_parameter('ekf_odom_topic', '/odom')
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('log_dir', str(Path.home() / 'r2_logs'))
        self.declare_parameter('encoder_file', '/dev/shm/r2_encoder')
        # write rows only while moving (or commanded), plus hold_s after motion stops
        self.declare_parameter('only_when_moving', True)
        self.declare_parameter('motion_threshold', 0.005)  # m/s or rad/s
        self.declare_parameter('hold_s', 2.0)
        self.declare_parameter('cmd_stale_s', 0.5)

        cmd_topic = self.get_parameter('cmd_topic').value
        reported_topic = self.get_parameter('reported_odom_topic').value
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        log_rate = float(self.get_parameter('log_rate').value)
        log_dir = Path(self.get_parameter('log_dir').value)
        self.encoder_file = self.get_parameter('encoder_file').value
        self.only_moving = bool(self.get_parameter('only_when_moving').value)
        self.thr = float(self.get_parameter('motion_threshold').value)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.cmd_stale_s = float(self.get_parameter('cmd_stale_s').value)
        self._cmd_t = 0.0
        self._last_motion_t = -1e9

        self._cmd = (None, None)  # vx, wz
        self._reported = (None, None)
        self._ekf = (None, None)

        # Commands and odom may differ in reliability; accept both.
        self.create_subscription(Twist, cmd_topic, self._on_cmd, _qos_keep_last())
        self.create_subscription(
            Odometry, reported_topic, self._on_reported, _qos_keep_last())
        self.create_subscription(
            Odometry, ekf_topic, self._on_ekf, _qos_keep_last())

        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._path = log_dir / f'odom_compare_{stamp}.csv'
        self._fh = open(self._path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._fh)
        self._writer.writerow([
            'time', 'cmd_vx', 'cmd_wz',
            'reported_vx', 'reported_wz',
            'ekf_vx', 'ekf_wz',
            'enc0', 'enc1', 'enc2', 'enc3',
        ])
        self._fh.flush()

        period = 1.0 / max(log_rate, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'Logging at {log_rate:.1f} Hz to {self._path} '
            f'(cmd={cmd_topic}, reported={reported_topic}, ekf={ekf_topic})'
        )

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_t = self.get_clock().now().nanoseconds / 1e9

    def _on_reported(self, msg):
        t = msg.twist.twist
        self._reported = (t.linear.x, t.angular.z)

    def _on_ekf(self, msg):
        t = msg.twist.twist
        self._ekf = (t.linear.x, t.angular.z)

    def _read_encoder(self):
        try:
            with open(self.encoder_file, 'r', encoding='utf-8') as f:
                parts = f.read().strip().split()
            if len(parts) >= 4:
                return parts[0], parts[1], parts[2], parts[3]
        except (OSError, ValueError):
            pass
        return '', '', '', ''

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.only_moving:
            def big(pair):
                return any(v is not None and abs(v) > self.thr for v in pair)
            cmd_fresh = (now - self._cmd_t) < self.cmd_stale_s
            if (cmd_fresh and big(self._cmd)) or big(self._reported) or big(self._ekf):
                self._last_motion_t = now
            elif now - self._last_motion_t > self.hold_s:
                return
        def fmt(pair, i):
            v = pair[i]
            return '' if v is None else f'{v:.6f}'
        enc = self._read_encoder()
        self._writer.writerow([
            f'{now:.6f}',
            fmt(self._cmd, 0), fmt(self._cmd, 1),
            fmt(self._reported, 0), fmt(self._reported, 1),
            fmt(self._ekf, 0), fmt(self._ekf, 1),
            *enc,
        ])
        self._fh.flush()

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OdomCompareLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
