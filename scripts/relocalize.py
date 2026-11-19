#!/usr/bin/env python3
"""Match current lidar to the saved occupancy map. Start Cartographer there, or fail."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
from r2_status import set_status  # noqa: E402


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_grid_from_yaml(yaml_path: Path) -> tuple[np.ndarray, float, float, float]:
    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    res = float(data['resolution'])
    ox, oy, _ = data['origin']
    img = Path(data['image'])
    if not img.is_absolute():
        img = yaml_path.with_name(data['image'])
    raw = img.read_bytes()
    lines = []
    i = 0
    while len(lines) < 3:
        nl = raw.find(b'\n', i)
        line = raw[i:nl]
        i = nl + 1
        if line.startswith(b'#'):
            continue
        lines.append(line)
    w, h = map(int, lines[1].split())
    grid = np.frombuffer(raw[i:i + w * h], dtype=np.uint8).reshape(h, w)
    occ = (grid < 120).astype(np.uint8)
    return occ, res, ox, oy


def grid_from_occupancy(msg: OccupancyGrid) -> tuple[np.ndarray, float, float, float]:
    w, h = int(msg.info.width), int(msg.info.height)
    arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
    occ = (arr >= 60).astype(np.uint8)
    return occ, float(msg.info.resolution), float(msg.info.origin.position.x), float(msg.info.origin.position.y)


def scan_xy(msg: LaserScan) -> np.ndarray:
    pts = []
    a = float(msg.angle_min)
    for r in msg.ranges:
        if 0.20 < r < min(8.0, float(msg.range_max)):
            pts.append((r * math.cos(a), r * math.sin(a)))
        a += float(msg.angle_increment)
    if len(pts) < 30:
        return np.zeros((0, 2), dtype=np.float32)
    arr = np.asarray(pts, dtype=np.float32)
    step = max(1, len(arr) // 80)
    return arr[::step]


def score_pose(occ: np.ndarray, res: float, ox: float, oy: float, pts: np.ndarray, x: float, y: float, yaw: float) -> float:
    h, w = occ.shape
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + pts[:, 0] * c - pts[:, 1] * s
    wy = y + pts[:, 0] * s + pts[:, 1] * c
    cx = np.floor((wx - ox) / res).astype(np.int32)
    cy = np.floor((wy - oy) / res).astype(np.int32)
    ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    if not np.any(ok):
        return 0.0
    return float(occ[cy[ok], cx[ok]].mean())


def search(occ: np.ndarray, res: float, ox: float, oy: float, pts: np.ndarray) -> tuple[float, float, float, float]:
    h, w = occ.shape
    best = (0.0, 0.0, 0.0, 0.0)
    step_xy = 0.20
    xs = np.arange(ox + 0.4, ox + w * res - 0.4, step_xy)
    ys = np.arange(oy + 0.4, oy + h * res - 0.4, step_xy)
    yaws = np.linspace(-math.pi, math.pi, 36, endpoint=False)
    for y in ys:
        for x in xs:
            ix = int((x - ox) / res)
            iy = int((y - oy) / res)
            if not (0 <= ix < w and 0 <= iy < h):
                continue
            # skip obviously occupied cells
            if occ[iy, ix]:
                continue
            for yaw in yaws:
                sc = score_pose(occ, res, ox, oy, pts, x, y, yaw)
                if sc > best[0]:
                    best = (sc, x, y, yaw)
    # refine
    sc0, x0, y0, yaw0 = best
    for dy in np.linspace(-0.15, 0.15, 7):
        for dx in np.linspace(-0.15, 0.15, 7):
            for dyaw in np.linspace(-math.radians(12), math.radians(12), 9):
                sc = score_pose(occ, res, ox, oy, pts, x0 + dx, y0 + dy, yaw0 + dyaw)
                if sc > best[0]:
                    best = (sc, x0 + dx, y0 + dy, yaw0 + dyaw)
    return best


class Relocalize(Node):
    def __init__(self, yaml_path: Path, min_score: float) -> None:
        super().__init__('r2_relocalize')
        self.yaml_path = yaml_path
        self.min_score = min_score
        self.scan = None
        self.grid_msg = None
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid_msg = msg


def start_cartographer_at(node: Node, x: float, y: float, yaw: float, lua_dir: str) -> bool:
    from cartographer_ros_msgs.srv import StartTrajectory
    cli = node.create_client(StartTrajectory, '/start_trajectory')
    if not cli.wait_for_service(timeout_sec=8.0):
        node.get_logger().error('start_trajectory 서비스가 없습니다')
        return False
    req = StartTrajectory.Request()
    req.configuration_directory = lua_dir
    req.configuration_basename = 'r2_cartographer.lua'
    req.use_initial_pose = True
    req.relative_to_trajectory_id = 0
    req.initial_pose = Pose()
    req.initial_pose.position.x = float(x)
    req.initial_pose.position.y = float(y)
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    req.initial_pose.orientation.z = qz
    req.initial_pose.orientation.w = qw
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    res = fut.result() if fut.done() else None
    if res is None:
        return False
    node.get_logger().info(f'start_trajectory id={res.trajectory_id} msg={res.status.message}')
    return int(getattr(res.status, 'code', 0)) == 0 or int(res.trajectory_id) >= 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--min-score', type=float, default=0.42)
    parser.add_argument('--lua-dir', default=str(Path(__file__).resolve().parent / 'params'))
    args = parser.parse_args()
    yaml_path = Path(args.map_yaml)

    set_status('맵에서 지금 위치를 찾는 중입니다…')
    rclpy.init()
    node = Relocalize(yaml_path, args.min_score)
    occ, res, ox, oy = load_grid_from_yaml(yaml_path)
    deadline = node.get_clock().now().nanoseconds + 12_000_000_000
    while node.scan is None and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.grid_msg is not None:
        occ, res, ox, oy = grid_from_occupancy(node.grid_msg)
    if node.scan is None:
        set_status('라이다 스캔이 없습니다.')
        print('라이다 스캔이 없습니다.', flush=True)
        return 2
    pts = scan_xy(node.scan)
    if len(pts) < 20:
        set_status('조금 돌아다녀봤지만 모르는 맵입니다.')
        print('스캔 점이 너무 적습니다.', flush=True)
        return 2
    print('맵과 라이다를 맞추는 중…', flush=True)
    score, x, y, yaw = search(occ, res, ox, oy, pts)
    print(f'맞춤 점수 {score:.2f}  위치 ({x:.2f}, {y:.2f})  yaw={math.degrees(yaw):.0f}°', flush=True)
    if score < args.min_score:
        set_status('조금 돌아다녀봤지만 모르는 맵입니다.')
        print('모르는 맵입니다. 순찰을 시작하지 않습니다.', flush=True)
        return 2
    if not start_cartographer_at(node, x, y, yaw, args.lua_dir):
        set_status('위치는 찾았지만 Cartographer 시작에 실패했습니다.')
        print('Cartographer 궤적 시작 실패', flush=True)
        return 1
    set_status('')
    print('맵에서 위치를 찾았습니다. 순찰을 시작합니다.', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
