#!/usr/bin/env python3
"""Closed-loop 3 m forward then reverse. Writes /tmp/r2_last_drive.json"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

# Lidar-only stop. No map required.
FRONT_STOP_M = 0.55
REAR_STOP_M = 0.55
CONE_DEG = 20.0
# 2D lidar has no height. A ~1 cm mat shows up as 0.3-0.5 m hits; skip those.
IGNORE_CLOSER_THAN = 0.50


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Drive3m(Node):
    def __init__(self):
        super().__init__('drive_3m_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data
        )
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos)
        self.pose = None
        self.start = None
        self.trace = []
        self.min_front = None
        self.min_rear = None
        self.blocked = None

    def on_scan(self, msg):
        front = []
        rear = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > IGNORE_CLOSER_THAN:
                deg = math.degrees(angle)
                # wrap to [-180, 180]
                while deg > 180:
                    deg -= 360
                while deg < -180:
                    deg += 360
                if abs(deg) <= CONE_DEG:
                    front.append(r)
                if abs(abs(deg) - 180) <= CONE_DEG:
                    rear.append(r)
            angle += msg.angle_increment
        self.min_front = min(front) if front else None
        self.min_rear = min(rear) if rear else None

    def on_odom(self, msg):
        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, y)
        if self.start is None:
            self.start = self.pose
        self.trace.append({
            't': time.time(),
            'x': p.x,
            'y': p.y,
            'yaw': y,
            'along': self.along_start(),
        })

    def stop(self):
        t = Twist()
        for _ in range(10):
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.04)

    def wait_odom(self, timeout=8.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def wait_scan(self, timeout=8.0):
        t0 = time.time()
        while self.min_front is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.min_front is not None

    def along_start(self):
        if not self.pose or not self.start:
            return 0.0
        dx = self.pose[0] - self.start[0]
        dy = self.pose[1] - self.start[1]
        yaw = self.start[2]
        return dx * math.cos(yaw) + dy * math.sin(yaw)

    def dist_from_start(self):
        if not self.pose or not self.start:
            return 0.0
        return math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])

    def drive(self, speed, target_along, timeout, label):
        print(f'=== {label} ===', flush=True)
        t0 = time.time()
        last_along = None
        stall = 0.0
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            along = self.along_start()
            if speed > 0 and along >= target_along:
                break
            if speed < 0 and along <= target_along:
                break
            if speed > 0 and self.min_front is not None and self.min_front < FRONT_STOP_M:
                self.blocked = f'front {self.min_front:.2f} m'
                print(f'STOP obstacle {self.blocked}', flush=True)
                break
            if speed < 0 and self.min_rear is not None and self.min_rear < REAR_STOP_M:
                self.blocked = f'rear {self.min_rear:.2f} m'
                print(f'STOP obstacle {self.blocked}', flush=True)
                break
            msg = Twist()
            msg.linear.x = float(speed)
            self.pub.publish(msg)
            if last_along is None or abs(along - last_along) >= 0.05:
                print(f'  along={along:.3f} m', flush=True)
                last_along = along
            if last_along is not None and abs(along - last_along) < 0.002:
                stall += 0.05
            else:
                stall = 0.0
            if stall > 4.0:
                print('STALL', flush=True)
                break
        self.stop()
        return self.along_start()


def main():
    rclpy.init()
    node = Drive3m()
    result = {
        'ok': False,
        'forward_m': None,
        'final_m': None,
        'error': None,
        'blocked': None,
    }
    try:
        if not node.wait_odom():
            result['error'] = 'no /odom'
            print('FAIL: no /odom', flush=True)
            return 2
        if not node.wait_scan():
            result['error'] = 'no /scan'
            print('FAIL: no /scan — cannot stop for obstacles', flush=True)
            return 2
        print(
            f'lidar front={node.min_front} m rear={node.min_rear}',
            flush=True,
        )
        result['start'] = {
            'x': node.start[0], 'y': node.start[1], 'yaw': node.start[2]
        }
        # Forward until obstacle (40 cm) or 15 m safety cap.
        node.blocked = None
        fwd = node.drive(0.15, 15.0, 150.0, 'FORWARD until obstacle')
        time.sleep(0.6)
        hit = node.blocked
        node.blocked = None
        back = node.drive(-0.15, 0.10, 150.0, 'REVERSE to start')
        node.stop()
        result['forward_m'] = round(fwd, 3)
        result['final_m'] = round(back, 3)
        result['final_xy'] = round(node.dist_from_start(), 3)
        result['ok'] = fwd >= 0.4
        result['blocked'] = hit or node.blocked
        result['min_front'] = node.min_front
        result['min_rear'] = node.min_rear
        result['trace_points'] = len(node.trace)
        # downsample trace for html
        step = max(1, len(node.trace) // 200)
        result['trace'] = node.trace[::step]
        print(
            f'RESULT forward={fwd:.3f} final={back:.3f} ok={result["ok"]}',
            flush=True,
        )
        return 0 if result['ok'] else 1
    except KeyboardInterrupt:
        result['error'] = 'interrupted'
        return 3
    finally:
        try:
            node.stop()
        except Exception:
            pass
        with open('/tmp/r2_last_drive.json', 'w', encoding='utf-8') as fh:
            json.dump(result, fh, ensure_ascii=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
