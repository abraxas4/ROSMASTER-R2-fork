# R2 Autonomy Enhancements

This document details the enhancements made to address specific requirements.

## 1. Path-Tracking Reverse Recovery ✅

**Implementation**: `scripts/path_reverse_recovery.py`

- Records recent driven path (5 second buffer, 0.05m sampling)
- When blocked going forward:
  1. Reverses along the recorded path (not straight back)
  2. Drives forward with steering bias (alternates LEFT/RIGHT)
- Direction alternation tested with unit tests (`test/test_direction_alternation.py`)
- Exposed as ROS 2 action (`r2_autonomy_msgs/action/PathReverseRecovery.action`)
- Integrated into map_mode launch

**Usage**:
```bash
ros2 action send_goal /path_reverse_recovery r2_autonomy_msgs/action/PathReverseRecovery "{trigger: true}"
```

## 2. Slip Detection with SLAM Pose ✅

**Implementation**: `scripts/slip_detector.py` (updated)

- Compares wheel odometry vs. **TF map->base_footprint** (from SLAM/AMCL)
- Independent of EKF (which fuses wheel encoders)
- Uses scan matching pose as ground truth
- Emergency stop after 3s of sustained slip

## 3. 0.40m Obstacle Threshold ✅

**Implementation**: `config/collision_monitor_params.yaml` + integrated into `nav2_params_ackermann.yaml`

- **ApproachZone**: 0.40m front/right-front → **SLOW DOWN** (approach action)
- **StopZone**: 0.20m all around → **EMERGENCY STOP**
- Prefers slowing over stopping
- Visualized polygons in RViz

## 4. Auto-Patrol Waypoints ✅

**Implementation**: `scripts/patrol_waypoint_generator.py`

- Samples free space from saved map
- Generates waypoints with minimum distance (1.5m default)
- Avoids obstacles with safety margin (0.50m)
- Loops automatically
- Integrated into patrol_mode launch

**Parameters**:
- `num_waypoints`: 10 (default)
- `min_distance_between_waypoints`: 1.5m
- `loop_patrol`: true

## 5. Keepout Zones ✅

**Implementation**: 
- `maps/keepout_mask.yaml` + `maps/keepout_mask.pgm`
- `config/keepout_filter.yaml`
- Integrated into global costmap

**How to draw keepout zones**:
1. Open `keepout_mask.pgm` in GIMP
2. Draw white (255) areas where robot should NOT go
3. Save as PGM
4. Robot will avoid these areas

## 6. Depth Camera in Costmap ✅

**Implementation**: Updated `config/nav2_params_ackermann.yaml`

- Added `/camera/depth/points` (`sensor_frame`: `camera_depth_optical_frame`) to voxel_layer
- Lidar: 3.0m range (main obstacle detection)
- Depth camera: 2.0m range (glass detection, 0.30-2.0m)
- Lightweight: only local costmap, 2.5m raytrace max
- Configured for Orbbec Astra Plus

## 7. Build & Test ✅

**Test infrastructure**:
- `launch/test_headless.launch.py`: Headless test with fake sensors
- `scripts/fake_scan_publisher.py`: Simulates LiDAR
- `scripts/fake_odom_publisher.py`: Simulates odometry
- `test/test_direction_alternation.py`: Unit tests for recovery logic
- `docker/Dockerfile.test`: ROS 2 Humble build environment

**Build verification**:
```bash
cd /workspace/code/yahboomcar_ros2_ws
colcon build --packages-select r2_autonomy
source install/setup.bash
ros2 launch r2_autonomy test_headless.launch.py
```

## Files Added

### Core Functionality
- `scripts/path_reverse_recovery.py` (180 lines)
- `scripts/patrol_waypoint_generator.py` (200 lines)
- `../r2_autonomy_msgs/action/PathReverseRecovery.action`

### Configuration
- `config/collision_monitor_params.yaml`
- `config/keepout_filter.yaml`
- `maps/keepout_mask.yaml` + `.pgm`

### Testing
- `launch/test_headless.launch.py`
- `scripts/fake_scan_publisher.py`
- `scripts/fake_odom_publisher.py`
- `test/test_direction_alternation.py`
- `docker/Dockerfile.test`

## Files Modified

- `scripts/slip_detector.py`: Now uses TF map->base instead of EKF
- `config/nav2_params_ackermann.yaml`: Added depth camera, collision monitor, keepout filter
- `launch/map_mode.launch.py`: Added path_reverse_recovery node
- `launch/patrol_mode.launch.py`: Added patrol_waypoint_generator node
- `CMakeLists.txt`: New scripts; action interfaces live in `r2_autonomy_msgs`
- `package.xml`: Added `<exec_depend>r2_autonomy_msgs</exec_depend>`

## Verification Checklist

✅ Python syntax (all scripts compile)
✅ XML validation (behavior trees)
✅ YAML validation (all configs)
✅ Unit tests pass (direction alternation)
✅ Action interface defined
✅ Launch files updated
✅ Package builds in ROS 2 Humble (see BUILD_RESULTS.md)
✅ Nodes start headlessly with fake data
✅ TF chain validated (map->odom->base_footprint->base_link->laser)

## Performance Considerations (Jetson Orin Nano)

- Depth camera limited to 2.0m, local costmap only
- Voxel layer resolution: 0.05m (can increase to 0.10m if needed)
- Path recording: 200 poses max (5s @ 40 Hz)
- Patrol waypoints: sample every 5th cell to reduce computation
- Collision monitor: 20 Hz update rate
