#!/bin/bash
# Stop ROS nodes commonly started for mapping / camera sessions.

echo "Stopping robot ROS stack..."

pkill -f '/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' 2>/dev/null || true
pkill -f 'yahboomcar_bringup_R2_launch.py' 2>/dev/null || true
pkill -f 'yahboomcar_bringup_X3_launch.py' 2>/dev/null || true
pkill -f 'laser_bringup_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_raw_launch.py' 2>/dev/null || true
pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -f 'yahboomcar_bringup/Ackman_driver' 2>/dev/null || true
pkill -f 'yahboomcar_bringup/Mcnamu_driver' 2>/dev/null || true
pkill -f 'yahboomcar_ctrl/yahboom_joy' 2>/dev/null || true
pkill -x joy_node 2>/dev/null || true
pkill -x imu_filter_madgwick_node 2>/dev/null || true
pkill -f 'base_node_R2' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'joint_state_publisher' 2>/dev/null || true
pkill -f 'astra_camera_node' 2>/dev/null || true
pkill -f 'astro_pro_plus.launch.xml' 2>/dev/null || true
pkill -f 'map_gmapping_4ros_launch.py' 2>/dev/null || true
pkill -f 'map_rtabmap_launch.py' 2>/dev/null || true
pkill -f 'rtabmap_sync_launch.py' 2>/dev/null || true
pkill -f 'rtabmap_viz_launch.py' 2>/dev/null || true
pkill -f 'slam_gmapping' 2>/dev/null || true
pkill -f 'rtabmap_slam.*/rtabmap' 2>/dev/null || true
pkill -f 'rtabmap_sync.*/rgbd_sync' 2>/dev/null || true

# Stop motors and silence board beeper.
python3 - <<'PY' 2>/dev/null || true
from Rosmaster_Lib import Rosmaster
car = Rosmaster()
car.set_car_motion(0, 0, 0)
car.set_beep(0)
PY

sleep 2
echo "Done."