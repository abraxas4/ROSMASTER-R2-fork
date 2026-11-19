#!/usr/bin/env python3
"""Reactive lidar(+depth) obstacle avoidance for R2 Ackermann.

Steers toward the freer side, backs up when the front is blocked.
Records map->base_link trajectory and saves the SLAM /map with the path at exit.
"""
import math, time, os, csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class LidarAvoid(Node):
    def __init__(self):
        super().__init__('lidar_avoid')
        p = self.declare_parameter
        self.v = p('speed', 0.12).value
        self.stop_d = p('stop_dist', 0.35).value
        self.slow_d = p('slow_dist', 0.80).value
        self.self_d = p('self_mask_dist', 0.18).value
        self.w_max = p('max_yaw_rate', 0.5).value
        self.run_time = p('run_time', 30.0).value
        self.out_dir = p('out_dir', os.path.expanduser('~/r2_runs')).value
        self.use_depth = p('use_depth', True).value
        self.front = self.left = self.right = None
        self.depth_front = None
        self.state = 'FWD'
        self.state_until = 0.0
        self.turn_sign = 1.0
        self.t0 = time.time()
        self.path = []
        self.map = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, mq)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_timer(0.1, self.step)
        self.create_timer(0.2, self.record)
        self.log = open(os.path.join(self.out_dir, 'avoid_log.csv'), 'w', newline='')
        self.logw = csv.writer(self.log)
        self.logw.writerow(['t', 'state', 'front', 'left', 'right', 'depth', 'vx', 'wz'])

    def on_scan(self, m):
        fr, lf, rt = [], [], []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < self.self_d:
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if abs(a) <= 25:
                fr.append(r)
            if 15 <= a <= 75:
                lf.append(min(r, 2.5))
            if -75 <= a <= -15:
                rt.append(min(r, 2.5))
        self.front = min(fr) if fr else 3.0
        self.left = float(np.percentile(lf, 30)) if lf else 0.0
        self.right = float(np.percentile(rt, 30)) if rt else 0.0

    def on_depth(self, m):
        a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        band = a[m.height // 3: m.height * 3 // 4, m.width // 4: m.width * 3 // 4]
        v = band[band > 0]
        self.depth_front = float(np.percentile(v, 3)) / 1000.0 if v.size > 500 else None

    def on_map(self, m):
        self.map = m

    def record(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.path.append((t.transform.translation.x, t.transform.translation.y))
        except Exception:
            pass

    def step(self):
        now = time.time()
        cmd = Twist()
        if self.front is None:
            self.pub.publish(cmd)
            return
        if now - self.t0 > self.run_time:
            self.pub.publish(cmd)
            raise SystemExit
        front = self.front
        if self.depth_front is not None:
            front = min(front, self.depth_front)
        side = 1.0 if self.left >= self.right else -1.0
        if self.state == 'BACK' and now < self.state_until:
            cmd.linear.x = -0.10
            cmd.angular.z = -self.turn_sign * self.w_max  # reverse: swing nose toward free side
        elif front < self.stop_d:
            self.state = 'BACK'
            self.turn_sign = side
            self.state_until = now + 1.5
            cmd.linear.x = -0.10
            cmd.angular.z = -side * self.w_max
        else:
            self.state = 'FWD'
            cmd.linear.x = self.v
            bal = (self.left - self.right)
            w = 0.6 * bal
            if front < self.slow_d:
                k = (self.slow_d - front) / (self.slow_d - self.stop_d)
                w = side * self.w_max * max(0.5, k)
                cmd.linear.x = self.v * 0.8
            cmd.angular.z = max(-self.w_max, min(self.w_max, w))
        self.pub.publish(cmd)
        self.logw.writerow([round(now - self.t0, 2), self.state, round(front, 2), round(self.left, 2),
                            round(self.right, 2), self.depth_front and round(self.depth_front, 2),
                            round(cmd.linear.x, 3), round(cmd.angular.z, 3)])

    def save(self):
        self.log.close()
        with open(os.path.join(self.out_dir, 'path.csv'), 'w') as f:
            for x, y in self.path:
                f.write(f'{x:.3f},{y:.3f}\n')
        if self.map is None:
            self.get_logger().warn('no /map received')
            return
        import cv2
        m = self.map
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(g.shape, 205, np.uint8)
        img[g == 0] = 255
        img[g > 50] = 0
        img = cv2.cvtColor(np.flipud(img), cv2.COLOR_GRAY2BGR)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        pts = [(int((x - ox) / res), m.info.height - 1 - int((y - oy) / res)) for x, y in self.path]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (0, 0, 255), 1)
        if pts:
            cv2.circle(img, pts[0], 3, (0, 180, 0), -1)
            cv2.circle(img, pts[-1], 3, (255, 0, 0), -1)
        scale = max(1, int(600 / max(img.shape[:2])))
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(self.out_dir, 'map_path.png'), img)
        self.get_logger().info(f'saved map {m.info.width}x{m.info.height} res={res} path={len(self.path)}')


def main():
    rclpy.init()
    n = LidarAvoid()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    for _ in range(5):
        n.pub.publish(Twist())
        time.sleep(0.05)
    n.save()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
