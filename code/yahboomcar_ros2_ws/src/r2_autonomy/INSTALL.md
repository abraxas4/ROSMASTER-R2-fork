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

`ros-humble-explore-lite` 는 apt 저장소에 없음. 소스에서 빌드함.

```bash
# 1) apt 확인 (패키지가 있으면 이걸로 끝)
sudo apt install ros-humble-explore-lite

# 2) 없으면 robo-friends/m-explore-ros2 의 main 을 클론 (humble 브랜치는 없음)
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws/src
git clone --depth 1 -b main https://github.com/robo-friends/m-explore-ros2.git

# Humble 에는 tf2_ros/*.hpp 가 없어서 패치를 적용
cd m-explore-ros2
git apply ~/ROSMASTER-R2-fork/scripts/patches/m-explore-ros2-humble-tf2_ros.patch

# explore_lite 와 메시지 패키지만 빌드 (map_merge 는 제외)
cd ~/ROSMASTER-R2-fork/code/yahboomcar_ros2_ws
colcon build --packages-select explore_lite_msgs explore_lite --symlink-install
source install/setup.bash
ros2 pkg executables explore_lite
```

이 저장소에는 m-explore-ros2 소스를 넣지 않음.

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
