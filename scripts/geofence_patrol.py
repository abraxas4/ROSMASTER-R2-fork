#!/usr/bin/env python3
"""Geofence-bounded waypoint patrol using Nav2 (dynamic obstacle avoidance via lidar)."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
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


def pgm_size(path: Path) -> tuple[int, int]:
    lines: list[str] = []
    with path.open('rb') as fh:
        for _ in range(20):
            line = fh.readline().decode('ascii', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            lines.append(line)
            if len(lines) >= 3:
                break
    if len(lines) < 3:
        raise RuntimeError(f'invalid PGM header: {path}')
    w, h = map(int, lines[1].split())
    return w, h


def geofence_from_map_yaml(yaml_path: Path, margin: float) -> dict:
    import yaml

    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    res = float(data['resolution'])
    ox, oy, _ = data['origin']
    img = yaml_path.with_name(data['image'])
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
    def __init__(self, geofence: dict):
        super().__init__('geofence_monitor')
        self.geofence = geofence
        self.outside = False
        self.pub_stop = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self._check)

    def _check(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except TransformException:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        g = self.geofence
        inside = g['min_x'] <= x <= g['max_x'] and g['min_y'] <= y <= g['max_y']
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
    args = parser.parse_args()

    reg = load_registry()
    map_id = reg.get('active_id')
    if not map_id:
        print('ERROR: no active map in registry')
        return 1

    entry = reg['maps'][map_id]
    export_yaml = entry.get('export_yaml')
    if not export_yaml or not Path(export_yaml).exists():
        print('ERROR: export map first: bash scripts/export_rtabmap_map.sh')
        return 1

    yaml_path = Path(export_yaml)
    GEOFENCE_DIR.mkdir(parents=True, exist_ok=True)
    PATROL_DIR.mkdir(parents=True, exist_ok=True)
    gf_path = GEOFENCE_DIR / f'{map_id}.json'
    wp_path = PATROL_DIR / f'{map_id}_waypoints.json'

    geofence = geofence_from_map_yaml(yaml_path, args.margin)
    gf_path.write_text(json.dumps(geofence, indent=2), encoding='utf-8')

    meta_path = PATROL_DIR / f'{map_id}_waypoints_meta.json'
    meta = {
        'margin': args.margin,
        'waypoint_inset': args.waypoint_inset,
        'version': 2,
    }
    if wp_path.exists() and meta_path.exists():
        saved = json.loads(meta_path.read_text(encoding='utf-8'))
        if saved == meta:
            waypoints = json.loads(wp_path.read_text(encoding='utf-8'))
        else:
            waypoints = waypoints_from_geofence(geofence, args.waypoint_inset)
            wp_path.write_text(json.dumps(waypoints, indent=2), encoding='utf-8')
            meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')
    else:
        waypoints = waypoints_from_geofence(geofence, args.waypoint_inset)
        wp_path.write_text(json.dumps(waypoints, indent=2), encoding='utf-8')
        meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')

    name = entry.get('display_name') or map_id
    print(f'Patrol map: {name}')
    print(f'Geofence: x=[{geofence["min_x"]:.2f},{geofence["max_x"]:.2f}] '
          f'y=[{geofence["min_y"]:.2f},{geofence["max_y"]:.2f}]')
    print(f'Waypoints: {len(waypoints)} (edit {wp_path} to customize)')
    print('Obstacle avoidance: Nav2 local costmap + /scan (people, furniture)')

    rclpy.init()
    monitor = GeofenceMonitor(geofence)
    navigator = BasicNavigator()
    print('Waiting for Nav2 + AMCL to become active...')
    navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
    print('Nav2 ready — starting patrol.')

    try:
        while rclpy.ok():
            for wp in waypoints:
                for _ in range(5):
                    rclpy.spin_once(monitor, timeout_sec=0.1)
                if monitor.outside:
                    print('Outside geofence — waiting to re-enter...')
                    time.sleep(1.0)
                    continue

                goal = pose_from_wp(wp)
                navigator.goToPose(goal)
                while not navigator.isTaskComplete():
                    rclpy.spin_once(monitor, timeout_sec=0.1)
                    if monitor.outside:
                        navigator.cancelTask()
                        break
                    time.sleep(0.1)

                if monitor.outside:
                    continue

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f'Reached ({wp["x"]:.2f}, {wp["y"]:.2f})')
                else:
                    print(f'Waypoint failed (result={result}), continuing...')

                time.sleep(args.pause)

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