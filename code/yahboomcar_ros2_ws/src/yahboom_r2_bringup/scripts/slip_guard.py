#!/usr/bin/env python3
"""Wheel-slip guard: if wheel odometry travel over the last `window` s is >= min_dist
and the scan-matched pose (map->base_link) moved < ratio of it, deactivate the
ackermann controller (motors stop) and keep it off. Max slip time = window (3 s)."""
import math, time, subprocess
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros


class SlipGuard(Node):
    def __init__(self):
        super().__init__('slip_guard')
        self.window = self.declare_parameter('window', 3.0).value
        self.min_dist = self.declare_parameter('min_dist', 0.10).value
        self.ratio = self.declare_parameter('ratio', 0.5).value
        self.wheel_travel = 0.0
        self.last_t = None
        self.hist = deque()
        self.tripped = False
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_subscription(Odometry, '/ackermann_steering_controller/odometry', self.on_wheel, 50)
        self.create_timer(0.1, self.check)

    def on_wheel(self, m):
        now = time.monotonic()
        if self.last_t is not None:
            self.wheel_travel += abs(m.twist.twist.linear.x) * min(now - self.last_t, 0.2)
        self.last_t = now

    def check(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        now = time.monotonic()
        self.hist.append((now, self.wheel_travel, t.transform.translation.x, t.transform.translation.y))
        while self.hist and now - self.hist[0][0] > self.window:
            self.hist.popleft()
        if self.tripped or now - self.hist[0][0] < self.window * 0.9:
            return
        _, w0, x0, y0 = self.hist[0]
        _, w1, x1, y1 = self.hist[-1]
        dw, dm = w1 - w0, math.hypot(x1 - x0, y1 - y0)
        if dw >= self.min_dist and dm < self.ratio * dw:
            self.tripped = True
            self.get_logger().error(f'WHEEL SLIP {self.window:.0f}s: wheel {dw:.2f} m, actual {dm:.2f} m -> controller off')
            subprocess.run(['ros2', 'control', 'set_controller_state', 'ackermann_steering_controller', 'inactive'])
            open('/tmp/r2_halt', 'w').write('slip\n')


def main():
    rclpy.init()
    rclpy.spin(SlipGuard())


if __name__ == '__main__':
    main()
