#!/usr/bin/env python3
"""Log cmd/steer/odom/imu so gear and bicycle numbers can be judged."""
from __future__ import annotations

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class AckermannCalibLog(Node):
    WHEELBASE = 0.25

    def __init__(self):
        super().__init__('ackermann_calib_log')
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = Path.home() / 'maps' / 'calib_logs'
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = out_dir / f'로그조향_{ts}.csv'
        self.fh = self.path.open('w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(
            [
                't_rel',
                'cmd_vx',
                'cmd_vy',
                'raw_vx',
                'raw_vy_x1000',
                'raw_wz_board',
                'delta_deg',
                'R_m',
                'wz_bicycle',
                'imu_wz',
                'ratio_imu_over_bicycle',
                'oraw_x',
                'oraw_y',
                'oraw_yaw_deg',
                'oraw_wz',
                'ekf_x',
                'ekf_y',
                'ekf_yaw_deg',
                'slam_x',
                'slam_y',
                'slam_yaw_deg',
                'phase',
            ]
        )
        self.cmd = (0.0, 0.0)
        self.raw = (0.0, 0.0, 0.0)
        self.imu_wz = 0.0
        self.oraw = None
        self.ekf = None
        self.slam = None
        self.t0 = time.time()
        self.last_write = 0.0
        rel = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Twist, '/vel_raw', self.on_raw, 50)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_raw', self.on_oraw, rel)
        self.create_subscription(Odometry, '/odom', self.on_ekf, rel)
        self.create_subscription(Odometry, '/odom_slam', self.on_slam, rel)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('calib csv → %s' % self.path)

    def on_cmd(self, m: Twist):
        self.cmd = (float(m.linear.x), float(m.linear.y))

    def on_raw(self, m: Twist):
        self.raw = (float(m.linear.x), float(m.linear.y), float(m.angular.z))

    def on_imu(self, m: Imu):
        self.imu_wz = float(m.angular_velocity.z)

    def on_oraw(self, m: Odometry):
        p = m.pose.pose.position
        self.oraw = (p.x, p.y, yaw_of(m.pose.pose.orientation), float(m.twist.twist.angular.z))

    def on_ekf(self, m: Odometry):
        p = m.pose.pose.position
        self.ekf = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def on_slam(self, m: Odometry):
        p = m.pose.pose.position
        self.slam = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def tick(self):
        now = time.time()
        if now - self.last_write < 0.049:
            return
        self.last_write = now
        vx, vy1000, wz_b = self.raw
        delta_deg = vy1000  # source treats this as degrees
        delta_rad = delta_deg * math.pi / 180.0
        if abs(math.tan(delta_rad)) < 1e-6:
            r_m = float('inf')
            wz_bic = 0.0
        else:
            r_m = self.WHEELBASE / math.tan(delta_rad)
            wz_bic = vx / r_m if r_m == r_m else 0.0
        ratio = (self.imu_wz / wz_bic) if abs(wz_bic) > 0.02 else float('nan')
        cmd_vx, cmd_vy = self.cmd
        phase = 'stop'
        if abs(cmd_vy) >= 0.015 and abs(cmd_vx) >= 0.05:
            phase = 'circle'
        elif abs(cmd_vx) >= 0.05:
            phase = 'straight'
        elif abs(cmd_vy) >= 0.015:
            phase = 'steer_only'
        ox = oy = oyaw = owz = float('nan')
        if self.oraw:
            ox, oy, oyaw, owz = self.oraw
            oyaw = math.degrees(oyaw)
        ex = ey = eyaw = float('nan')
        if self.ekf:
            ex, ey, eyaw = self.ekf
            eyaw = math.degrees(eyaw)
        sx = sy = syaw = float('nan')
        if self.slam:
            sx, sy, syaw = self.slam
            syaw = math.degrees(syaw)
        self.w.writerow(
            [
                round(now - self.t0, 3),
                round(cmd_vx, 4),
                round(cmd_vy, 5),
                round(vx, 4),
                round(vy1000, 4),
                round(wz_b, 5),
                round(delta_deg, 3),
                (round(r_m, 4) if r_m != float('inf') else ''),
                round(wz_bic, 5),
                round(self.imu_wz, 5),
                (round(ratio, 4) if ratio == ratio else ''),
                round(ox, 4) if ox == ox else '',
                round(oy, 4) if oy == oy else '',
                round(oyaw, 2) if oyaw == oyaw else '',
                round(owz, 5) if owz == owz else '',
                round(ex, 4) if ex == ex else '',
                round(ey, 4) if ey == ey else '',
                round(eyaw, 2) if eyaw == eyaw else '',
                round(sx, 4) if sx == sx else '',
                round(sy, 4) if sy == sy else '',
                round(syaw, 2) if syaw == syaw else '',
                phase,
            ]
        )
        self.fh.flush()

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AckermannCalibLog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
