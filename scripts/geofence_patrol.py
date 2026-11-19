#!/usr/bin/env python3
"""Geofence-bounded waypoint patrol using Nav2 (dynamic obstacle avoidance via lidar)."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


HOME = Path.home()
MAPS_DIR = HOME / 'maps'
REGISTRY_PATH = MAPS_DIR / 'registry.json'
PATROL_DIR = MAPS_DIR / 'patrol'
GEOFENCE_DIR = MAPS_DIR / 'geofence'


def load_registry() -> dict:
    if not REGISTRY_PATH.exists():
        return {'active_id': None, 'maps': {}}
    return json.loads(REGISTRY_PATH.read_text(encoding='utf-8'))


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    with path.open('rb') as fh:
        header: list[str] = []
        while len(header) < 3:
            line = fh.readline().decode('ascii', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            header.append(line)
        w, h = map(int, header[1].split())
        grid = list(fh.read(w * h))
    if len(grid) < w * h:
        raise RuntimeError(f'invalid PGM payload: {path}')
    return w, h, grid


def pgm_size(path: Path) -> tuple[int, int]:
    w, h, _ = read_pgm(path)
    return w, h


def pgm_path_from_yaml(yaml_path: Path, image: str) -> Path:
    p = Path(image)
    if p.is_absolute():
        return p
    return yaml_path.with_name(image)


def waypoints_from_map_yaml(yaml_path: Path, clearance_m: float = 0.5) -> list[dict]:
    """Pick patrol goals only on known-free map cells (not unknown/occupied edges)."""
    import yaml

    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    res = float(data['resolution'])
    ox, oy, _ = data['origin']
    pgm_path = pgm_path_from_yaml(yaml_path, data['image'])
    w, h, grid = read_pgm(pgm_path)
    radius_cells = max(1, int(math.ceil(clearance_m / res)))

    def cell_free(cx: int, cy: int) -> bool:
        if not (0 <= cx < w and 0 <= cy < h):
            return False
        if grid[cy * w + cx] < 250:
            return False
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < h) or grid[ny * w + nx] < 250:
                    return False
        return True

    free_pts: list[tuple[float, float]] = []
    for cy in range(h):
        for cx in range(w):
            if cell_free(cx, cy):
                free_pts.append((ox + (cx + 0.5) * res, oy + (cy + 0.5) * res))

    if not free_pts:
        raise RuntimeError('no free map cells with requested clearance')

    xs = [p[0] for p in free_pts]
    ys = [p[1] for p in free_pts]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0

    def nearest_free(x: float, y: float) -> tuple[float, float]:
        best = free_pts[0]
        best_d = float('inf')
        for fx, fy in free_pts:
            d = (fx - x) ** 2 + (fy - y) ** 2
            if d < best_d:
                best_d = d
                best = (fx, fy)
        return best

    targets = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (cx, cy)]
    waypoints: list[dict] = []
    seen: set[tuple[float, float]] = set()
    for x, y in targets:
        fx, fy = nearest_free(x, y)
        key = (round(fx, 2), round(fy, 2))
        if key in seen:
            continue
        seen.add(key)
        waypoints.append({'x': fx, 'y': fy, 'yaw': 0.0})
    return waypoints


def geofence_from_map_yaml(yaml_path: Path, margin: float) -> dict:
    import yaml

    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    res = float(data['resolution'])
    ox, oy, _ = data['origin']
    img = pgm_path_from_yaml(yaml_path, data['image'])
    w, h = pgm_size(img)
    x0 = ox + margin
    y0 = oy + margin
    x1 = ox + w * res - margin
    y1 = oy + h * res - margin
    return {
        'frame': 'map',
        'margin_m': margin,
        'min_x': min(x0, x1),
        'max_x': max(x0, x1),
        'min_y': min(y0, y1),
        'max_y': max(y0, y1),
    }


def yaml_map_bounds(yaml_path: Path) -> dict:
    import yaml

    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    res = float(data['resolution'])
    ox, oy, _ = data['origin']
    img = pgm_path_from_yaml(yaml_path, data['image'])
    w, h = pgm_size(img)
    return {
        'min_x': ox,
        'max_x': ox + w * res,
        'min_y': oy,
        'max_y': oy + h * res,
    }


def geofence_from_occupancy(msg: OccupancyGrid, margin: float) -> dict:
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    w = int(msg.info.width)
    h = int(msg.info.height)
    x0 = ox + margin
    y0 = oy + margin
    x1 = ox + w * res - margin
    y1 = oy + h * res - margin
    return {
        'frame': 'map',
        'margin_m': margin,
        'min_x': min(x0, x1),
        'max_x': max(x0, x1),
        'min_y': min(y0, y1),
        'max_y': max(y0, y1),
    }


def _cell_clear(data: list[int], w: int, h: int, cx: int, cy: int, radius: int) -> bool:
    if not (0 <= cx < w and 0 <= cy < h):
        return False
    v = data[cy * w + cx]
    if v < 0 or v > 25:
        return False
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                return False
            nv = data[ny * w + nx]
            if nv < 0 or nv >= 65:
                return False
    return True


def explore_waypoints_from_occupancy(
    msg: OccupancyGrid,
    known: list[dict],
    init_bounds: dict,
    clearance_m: float = 0.5,
    max_goals: int = 3,
    min_dist: float = 1.0,
) -> list[dict]:
    """Frontier goals in newly expanded map, not the already-known interior."""
    w = int(msg.info.width)
    h = int(msg.info.height)
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    data = list(msg.data)
    radius = max(1, int(math.ceil(clearance_m / res)))
    nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1))

    def world(cx: int, cy: int) -> tuple[float, float]:
        return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

    def is_new_area(x: float, y: float) -> bool:
        edge = 0.45
        if x < init_bounds['min_x'] or x > init_bounds['max_x']:
            return True
        if y < init_bounds['min_y'] or y > init_bounds['max_y']:
            return True
        return (
            x - init_bounds['min_x'] < edge
            or init_bounds['max_x'] - x < edge
            or y - init_bounds['min_y'] < edge
            or init_bounds['max_y'] - y < edge
        )

    def far_from_known(x: float, y: float) -> bool:
        for wp in known:
            if (x - wp['x']) ** 2 + (y - wp['y']) ** 2 < min_dist * min_dist:
                return False
        return True

    bins: dict[tuple[int, int], list[tuple[float, float]]] = {}
    bin_m = 0.7
    for cy in range(1, h - 1):
        for cx in range(1, w - 1):
            v = data[cy * w + cx]
            if v < 0 or v > 25:
                continue
            if not any(
                data[(cy + dy) * w + (cx + dx)] < 0
                for dx, dy in nbrs
            ):
                continue
            if not _cell_clear(data, w, h, cx, cy, radius):
                continue
            x, y = world(cx, cy)
            if not is_new_area(x, y) or not far_from_known(x, y):
                continue
            key = (int(math.floor(x / bin_m)), int(math.floor(y / bin_m)))
            bins.setdefault(key, []).append((x, y))

    ranked = sorted(bins.values(), key=len, reverse=True)
    goals: list[dict] = []
    seen: set[tuple[float, float]] = set()
    for pts in ranked:
        if len(goals) >= max_goals:
            break
        sx = sum(p[0] for p in pts) / len(pts)
        sy = sum(p[1] for p in pts) / len(pts)
        key = (round(sx, 1), round(sy, 1))
        if key in seen:
            continue
        if not far_from_known(sx, sy):
            continue
        seen.add(key)
        yaw = math.atan2(sy - (init_bounds['min_y'] + init_bounds['max_y']) / 2.0,
                         sx - (init_bounds['min_x'] + init_bounds['max_x']) / 2.0)
        goals.append({'x': sx, 'y': sy, 'yaw': yaw, 'kind': 'explore'})
    return goals


def waypoints_from_geofence(gf: dict, inset: float = 0.8) -> list[dict]:
    """Place patrol corners inset from geofence so the 0.5 m body radius clears walls."""
    x0 = gf['min_x'] + inset
    x1 = gf['max_x'] - inset
    y0 = gf['min_y'] + inset
    y1 = gf['max_y'] - inset
    if x1 <= x0 or y1 <= y0:
        raise RuntimeError(
            f'Geofence too small for inset {inset} m '
            f'(x=[{gf["min_x"]:.2f},{gf["max_x"]:.2f}], '
            f'y=[{gf["min_y"]:.2f},{gf["max_y"]:.2f}])'
        )
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    pts = [
        (x0, y0, 0.0),
        (x1, y0, math.pi / 2),
        (x1, y1, math.pi),
        (x0, y1, -math.pi / 2),
        (cx, cy, 0.0),
    ]
    return [{'x': x, 'y': y, 'yaw': yaw} for x, y, yaw in pts]


class GeofenceMonitor(Node):
    def __init__(
        self,
        geofence: dict,
        known_waypoints: list[dict],
        init_bounds: dict,
        margin: float,
        explore_new: bool,
    ):
        super().__init__('geofence_monitor')
        self.geofence = geofence
        self.known_waypoints = known_waypoints
        self.init_bounds = init_bounds
        self.margin = margin
        self.explore_new = explore_new
        self.outside = False
        self.blocked = False
        self._lock = threading.Lock()
        self._explore: list[dict] = []
        self._map_sig: tuple | None = None
        self.pub_stop = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self._check)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        vol_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, vol_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def explore_waypoints(self) -> list[dict]:
        with self._lock:
            return list(self._explore)

    def _on_scan(self, msg: LaserScan) -> None:
        near, far = 0.22, 0.42
        half = math.radians(28.0)
        hits = 0
        angle = float(msg.angle_min)
        for r in msg.ranges:
            a = angle
            angle += float(msg.angle_increment)
            if abs(a) > half:
                continue
            if near < r < far:
                hits += 1
        hit = hits >= 4
        if hit and not self.blocked:
            self.get_logger().warn('앞이 가까워 전진하지 않습니다')
            Path('/tmp/r2_status.txt').write_text('앞이 가까워 전진하지 않습니다.\n', encoding='utf-8')
            self.pub_stop.publish(Twist())
            self.blocked = True
        elif not hit:
            if self.blocked:
                Path('/tmp/r2_status.txt').write_text('\n', encoding='utf-8')
            self.blocked = False

    def _on_map(self, msg: OccupancyGrid) -> None:
        sig = (
            int(msg.info.width),
            int(msg.info.height),
            round(float(msg.info.origin.position.x), 2),
            round(float(msg.info.origin.position.y), 2),
            hash(bytes(msg.data[:: max(1, len(msg.data) // 400)])),
        )
        if sig == self._map_sig:
            return
        self._map_sig = sig
        live = geofence_from_occupancy(msg, self.margin)
        with self._lock:
            self.geofence.update(live)
            if self.explore_new:
                self._explore = explore_waypoints_from_occupancy(
                    msg, self.known_waypoints, self.init_bounds
                )
        if self.explore_new:
            n = len(self._explore)
            self.get_logger().info(
                f'live map {msg.info.width}x{msg.info.height} '
                f'geofence x=[{live["min_x"]:.2f},{live["max_x"]:.2f}] '
                f'explore={n}'
            )

    def _check(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except TransformException:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        with self._lock:
            g = dict(self.geofence)
        inside = g['min_x'] <= x <= g['max_x'] and g['min_y'] <= y <= g['max_y']
        if self.blocked:
            self.pub_stop.publish(Twist())
        if not inside and not self.outside:
            self.get_logger().warn(
                f'GEOFENCE BREACH at ({x:.2f}, {y:.2f}) — stopping robot'
            )
            self.pub_stop.publish(Twist())
            self.outside = True
        elif inside:
            self.outside = False


def pose_from_wp(wp: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = wp['x']
    pose.pose.position.y = wp['y']
    qx, qy, qz, qw = yaw_to_quat(wp.get('yaw', 0.0))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main() -> int:
    parser = argparse.ArgumentParser(description='Geofence patrol with Nav2')
    parser.add_argument('--margin', type=float, default=1.0, help='Inset from map edge (m)')
    parser.add_argument('--waypoint-inset', type=float, default=0.8,
                        help='Extra inset for patrol corners inside geofence (m)')
    parser.add_argument('--loop', action='store_true', default=True)
    parser.add_argument('--pause', type=float, default=3.0, help='Seconds at each waypoint')
    parser.add_argument(
        '--localizer',
        default='amcl',
        help='Nav2 localizer node. Use bt_navigator when Cartographer owns map->odom.',
    )
    parser.add_argument(
        '--explore-new',
        action='store_true',
        default=True,
        help='After known-area waypoints, also go to newly expanded map frontiers.',
    )
    parser.add_argument('--no-explore-new', dest='explore_new', action='store_false')
    args = parser.parse_args()

    reg = load_registry()
    map_id = reg.get('active_id')
    if not map_id:
        print('ERROR: no active map in registry')
        return 1

    entry = reg['maps'][map_id]
    export_yaml = entry.get('export_yaml')
    if not export_yaml or not Path(export_yaml).exists():
        print('ERROR: export map first: bash scripts/save_cartographer_map.sh')
        return 1

    yaml_path = Path(export_yaml)
    GEOFENCE_DIR.mkdir(parents=True, exist_ok=True)
    PATROL_DIR.mkdir(parents=True, exist_ok=True)
    gf_path = GEOFENCE_DIR / f'{map_id}.json'
    wp_path = PATROL_DIR / f'{map_id}_waypoints.json'

    init_bounds = yaml_map_bounds(yaml_path)
    geofence = geofence_from_map_yaml(yaml_path, args.margin)
    gf_path.write_text(json.dumps(geofence, indent=2), encoding='utf-8')

    meta_path = PATROL_DIR / f'{map_id}_waypoints_meta.json'
    meta = {
        'margin': args.margin,
        'waypoint_inset': args.waypoint_inset,
        'clearance_m': 0.5,
        'source': 'free_map_cells',
        'version': 3,
    }
    if wp_path.exists() and meta_path.exists():
        saved = json.loads(meta_path.read_text(encoding='utf-8'))
        if saved == meta:
            waypoints = json.loads(wp_path.read_text(encoding='utf-8'))
        else:
            waypoints = waypoints_from_map_yaml(yaml_path, clearance_m=0.5)
            wp_path.write_text(json.dumps(waypoints, indent=2), encoding='utf-8')
            meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')
    else:
        waypoints = waypoints_from_map_yaml(yaml_path, clearance_m=0.5)
        wp_path.write_text(json.dumps(waypoints, indent=2), encoding='utf-8')
        meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')

    name = entry.get('display_name') or map_id
    print(f'Patrol map: {name}')
    print(f'Known geofence: x=[{geofence["min_x"]:.2f},{geofence["max_x"]:.2f}] '
          f'y=[{geofence["min_y"]:.2f},{geofence["max_y"]:.2f}]')
    print(f'Known waypoints: {len(waypoints)} (edit {wp_path} to customize)')
    print('Explore new map: ' + ('on' if args.explore_new else 'off'))
    print('Obstacle avoidance: Nav2 local costmap + /scan (people, furniture)')

    rclpy.init()
    monitor = GeofenceMonitor(
        geofence,
        known_waypoints=waypoints,
        init_bounds=init_bounds,
        margin=args.margin,
        explore_new=args.explore_new,
    )
    navigator = BasicNavigator()
    if args.localizer == 'amcl':
        print('Waiting for Nav2 + AMCL...')
        navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
    else:
        print('Nav2 is already up — starting patrol (no AMCL wait).')
    print('Nav2 ready — starting patrol.')

    try:
        while rclpy.ok():
            cycle = list(waypoints)
            extra = monitor.explore_waypoints()
            if extra:
                print(f'New-area goals this loop: {len(extra)}')
                cycle.extend(extra)
            for wp in cycle:
                for _ in range(5):
                    rclpy.spin_once(monitor, timeout_sec=0.1)
                if monitor.outside:
                    print('지오펜스 밖입니다. 다시 들어올 때까지 기다립니다.')
                    time.sleep(1.0)
                    continue
                if monitor.blocked:
                    print('앞이 막혀 멈춥니다. 밀지 않습니다.')
                    time.sleep(1.0)
                    continue

                goal = pose_from_wp(wp)
                navigator.goToPose(goal)
                while not navigator.isTaskComplete():
                    rclpy.spin_once(monitor, timeout_sec=0.1)
                    if monitor.outside or monitor.blocked:
                        navigator.cancelTask()
                        break
                    time.sleep(0.1)

                if monitor.outside or monitor.blocked:
                    continue

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    kind = '새 구역' if wp.get('kind') == 'explore' else '아는 곳'
                    print(f'{kind} 도착 ({wp["x"]:.2f}, {wp["y"]:.2f})')
                    time.sleep(args.pause)
                else:
                    print(f'목표 실패 (result={result}), 바로 다음으로 갑니다.')

            if not args.loop:
                break
    except KeyboardInterrupt:
        print('Patrol stopped.')
    finally:
        navigator.cancelTask()
        stop = Twist()
        monitor.pub_stop.publish(stop)
        monitor.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())