#!/usr/bin/env python3
"""Save one raw sample of each mapping sensor + occupancy map PNG."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float32

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

OUT = Path('/tmp/r2_samples')


def wait_one(node, msg_type, topic, qos=10, timeout=8.0):
    box = {'m': None}

    def cb(msg):
        box['m'] = msg

    sub = node.create_subscription(msg_type, topic, cb, qos)
    t0 = node.get_clock().now()
    while box['m'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - t0).nanoseconds > timeout * 1e9:
            break
    node.destroy_subscription(sub)
    return box['m']


def dump_sensors(node):
    OUT.mkdir(parents=True, exist_ok=True)
    scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    img_qos = qos_profile_sensor_data
    out = {}

    scan = wait_one(node, LaserScan, '/scan', scan_qos)
    if scan:
        ranges = [float(r) if math.isfinite(r) else None for r in list(scan.ranges)[:40]]
        finite = [r for r in scan.ranges if math.isfinite(r) and r > 0.05]
        out['scan'] = {
            'topic': '/scan',
            'frame_id': scan.header.frame_id,
            'angle_min_deg': math.degrees(scan.angle_min),
            'angle_max_deg': math.degrees(scan.angle_max),
            'angle_increment_deg': math.degrees(scan.angle_increment),
            'range_min': scan.range_min,
            'range_max': scan.range_max,
            'n_beams': len(scan.ranges),
            'first_40_ranges_m': ranges,
            'min_valid_m': min(finite) if finite else None,
            'max_valid_m': max(finite) if finite else None,
        }

    imu = wait_one(node, Imu, '/imu/data_raw')
    if imu:
        out['imu_raw'] = {
            'topic': '/imu/data_raw',
            'frame_id': imu.header.frame_id,
            'linear_acceleration': {
                'x': imu.linear_acceleration.x,
                'y': imu.linear_acceleration.y,
                'z': imu.linear_acceleration.z,
            },
            'angular_velocity': {
                'x': imu.angular_velocity.x,
                'y': imu.angular_velocity.y,
                'z': imu.angular_velocity.z,
            },
        }

    odom = wait_one(node, Odometry, '/odom')
    if odom:
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        t = odom.twist.twist
        out['odom'] = {
            'topic': '/odom',
            'frame_id': odom.header.frame_id,
            'child_frame_id': odom.child_frame_id,
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation_xyzw': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
            'twist_linear_x': t.linear.x,
            'twist_angular_z': t.angular.z,
        }

    info = wait_one(node, CameraInfo, '/camera/color/camera_info')
    if info:
        out['camera_info'] = {
            'topic': '/camera/color/camera_info',
            'width': info.width,
            'height': info.height,
            'distortion_model': info.distortion_model,
            'D': list(info.d),
            'K': list(info.k),
            'R': list(info.r),
            'P': list(info.p),
        }

    vol = wait_one(node, Float32, '/voltage')
    if vol:
        out['voltage'] = {'topic': '/voltage', 'volts': vol.data}

    rgb = wait_one(node, Image, '/camera/color/image_raw', img_qos)
    if rgb and CvBridge:
        cv = CvBridge().imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        import cv2
        path = OUT / 'sample_rgb.jpg'
        cv2.imwrite(str(path), cv)
        out['rgb_image'] = {
            'topic': '/camera/color/image_raw',
            'encoding': rgb.encoding,
            'width': rgb.width,
            'height': rgb.height,
            'file': str(path),
        }

    depth = wait_one(node, Image, '/camera/depth/image_raw', img_qos)
    if depth and CvBridge:
        cv = CvBridge().imgmsg_to_cv2(depth, desired_encoding='passthrough')
        arr = np.array(cv)
        out['depth_image'] = {
            'topic': '/camera/depth/image_raw',
            'encoding': depth.encoding,
            'width': depth.width,
            'height': depth.height,
            'dtype': str(arr.dtype),
            'min': float(np.nanmin(arr)) if arr.size else None,
            'max': float(np.nanmax(arr)) if arr.size else None,
            'center_pixel': float(arr[arr.shape[0] // 2, arr.shape[1] // 2]),
        }
        # visualize
        vis = arr.astype(np.float32)
        if vis.max() > 20:  # likely mm
            vis = vis / 1000.0
        vis = np.clip(vis, 0, 5.0)
        vis = (vis / 5.0 * 255).astype(np.uint8)
        import cv2
        path = OUT / 'sample_depth.png'
        cv2.imwrite(str(path), vis)
        out['depth_image']['preview'] = str(path)

    (OUT / 'sensors.json').write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding='utf-8')
    print('wrote', OUT / 'sensors.json')


def dump_map(node):
    OUT.mkdir(parents=True, exist_ok=True)
    grid = wait_one(node, OccupancyGrid, '/map', timeout=12.0)
    if not grid:
        print('no /map')
        return
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    data = np.array(grid.data, dtype=np.int16).reshape((h, w))
    # occupancy: -1 unknown, 0 free, 100 occupied
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[data < 0] = (200, 200, 200)
    rgb[data == 0] = (255, 255, 255)
    rgb[data > 50] = (30, 30, 30)
    mid = (data > 0) & (data <= 50)
    rgb[mid] = (180, 180, 180)
    from PIL import Image
    img = Image.fromarray(rgb, 'RGB').transpose(Image.FLIP_TOP_BOTTOM)
    # upscale if tiny
    if max(w, h) < 400:
        scale = max(2, 400 // max(w, h))
        img = img.resize((w * scale, h * scale), Image.NEAREST)
    path = OUT / 'result_map.png'
    img.save(path)
    meta = {
        'topic': '/map',
        'width': w,
        'height': h,
        'resolution_m': res,
        'origin_xy': [ox, oy],
        'unknown': int((data < 0).sum()),
        'free': int((data == 0).sum()),
        'occupied': int((data > 50).sum()),
        'file': str(path),
    }
    (OUT / 'map.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    print('wrote', path, meta)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-only', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('dump_samples')
    try:
        if args.map_only:
            dump_map(node)
        else:
            dump_sensors(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
