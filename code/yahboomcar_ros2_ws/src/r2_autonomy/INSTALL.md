# R2 Autonomy Installation Guide

## System Requirements

- Jetson Orin Nano (Ubuntu 22.04)
- ROS 2 Humble
- At least 4GB free disk space

## Installation Steps

### 1. Install ROS 2 Dependencies

```bash
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws/src/r2_autonomy
bash install_dependencies.sh
```

### 2. Build the Package

```bash
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws
colcon build --packages-select r2_autonomy --symlink-install
source install/setup.bash
```

### 3. Verify Installation

```bash
ros2 pkg list | grep r2_autonomy
ros2 launch r2_autonomy --show-args
```

## Optional: Install Frontier Exploration

If you want to use autonomous frontier exploration:

```bash
# Clone m-explore-ros2 (if not already installed)
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws/src
git clone -b humble https://github.com/robo-friends/m-explore-ros2.git
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws
colcon build --packages-select explore_lite --symlink-install
```

## Troubleshooting

### Missing Dependencies

If you get errors about missing packages, install them manually:

```bash
sudo apt install ros-humble-<package-name>
```

### Build Errors

Make sure yahboom_r2_bringup and yahboomcar_description are built first:

```bash
colcon build --packages-select yahboomcar_description yahboom_r2_bringup
```

## Next Steps

See README.md for usage instructions.
