#!/usr/bin/env python3
"""Halt if wheels slip about one revolution versus lidar pose.

Compares commanded / MCU wheel travel to TF pose (map, else odom).
R2 wheel diameter is ~65 mm, so one revolution is ~0.20 m.
"""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

HALT = Path('/tmp/r2_halt')
STATUS = Path('/tmp/r2_status.txt')
# 65 mm 바퀴 둘레. 한 바퀴만큼 헛돌면 중지.
WHEEL_REV_M = math.pi * 0.065
WINDOW_S = 2.0
CMD_MIN = 0.04
MAX_DT = 0.20


class SlipStop(Node):
    def __init__(self) -> None:
        super().__init__('slip_stop')
        self.cmd_vx = 0.0
        self.board_vx = 0.0
        self.last_pose = None
        self.last_frame = ''
        self.last_t = time.monotonic()
        self.samples: deque[tuple[float, float, float]] = deque()
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Twist, '/vel_raw', self._on_board, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_vx = float(msg.linear.x)

    def _on_board(self, msg: Twist) -> None:
        self.board_vx = float(msg.linear.x)

    def _lookup_pose(self) -> tuple[str, float, float] | None:
        for parent in ('map', 'odom'):
            try:
                tf = self.buf.lookup_transform(parent, 'base_footprint', rclpy.time.Time())
            except TransformException:
                continue
            t = tf.transform.translation
            return parent, float(t.x), float(t.y)
        return None

    def _halt(self) -> None:
        try:
            HALT.write_text('slip\n', encoding='utf-8')
            STATUS.write_text('바퀴가 헛돌아 중지합니다.\n', encoding='utf-8')
        except OSError:
            pass
        self.get_logger().error('바퀴 슬립 1회전 — 정지')
        self.pub.publish(Twist())

    def _tick(self) -> None:
        if HALT.exists():
            self.pub.publish(Twist())
            return
        now = time.monotonic()
        dt = max(1e-3, min(MAX_DT, now - self.last_t))
        self.last_t = now
        pose = self._lookup_pose()
        if pose is None:
            return
        frame, x, y = pose
        if self.last_pose is None or frame != self.last_frame:
            self.last_pose = (x, y)
            self.last_frame = frame
            self.samples.clear()
            return
        pose_inc = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y)
        wheel_vx = max(abs(self.cmd_vx), abs(self.board_vx))
        if wheel_vx < CMD_MIN:
            self.samples.clear()
            return
        self.samples.append((now, wheel_vx * dt, pose_inc))
        cutoff = now - WINDOW_S
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        wheel_m = sum(s[1] for s in self.samples)
        pose_m = sum(s[2] for s in self.samples)
        if wheel_m - pose_m >= WHEEL_REV_M:
            self._halt()


def main() -> int:
    rclpy.init()
    node = SlipStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
