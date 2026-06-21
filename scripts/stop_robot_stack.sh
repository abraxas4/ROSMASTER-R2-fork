#!/bin/bash
# Stop ROS nodes commonly started for mapping / camera sessions.

echo "Stopping robot ROS stack..."

pkill -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' 2>/dev/null || true
pkill -f 'yahboomcar_bringup_R2_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_raw_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -f 'Ackman_driver_R2' 2>/dev/null || true
pkill -f 'astra_camera_node' 2>/dev/null || true
pkill -f 'astro_pro_plus.launch.xml' 2>/dev/null || true
pkill -f 'map_gmapping_4ros_launch.py' 2>/dev/null || true
pkill -f 'map_rtabmap_launch.py' 2>/dev/null || true
pkill -f 'rtabmap_sync_launch.py' 2>/dev/null || true
pkill -f 'slam_gmapping' 2>/dev/null || true
pkill -f 'rtabmap_slam.*/rtabmap' 2>/dev/null || true
pkill -f 'rtabmap_sync.*/rgbd_sync' 2>/dev/null || true

sleep 2
echo "Done."