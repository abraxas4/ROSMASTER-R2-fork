#!/usr/bin/env bash
# Deploy yahboom_joy_R2.py from fork to rover runtime paths and restart.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOY_SRC="${REPO_ROOT}/code/yahboomcar_ros2_ws/src/yahboomcar_ctrl/yahboomcar_ctrl/yahboom_joy_R2.py"
ROVER="${ROVER:-rover}"

REMOTE_SRC=~/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_ctrl/yahboomcar_ctrl/yahboom_joy_R2.py
REMOTE_INSTALL=~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.10/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py

echo "Deploying ${JOY_SRC} -> ${ROVER}"
scp "${JOY_SRC}" "${ROVER}:${REMOTE_SRC}"
scp "${JOY_SRC}" "${ROVER}:${REMOTE_INSTALL}"

ssh "${ROVER}" bash -s <<'REMOTE'
set -euo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-28}"
pkill -f 'yahboom_joy_R2' 2>/dev/null || true
sleep 1
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
nohup env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ros2 run yahboomcar_ctrl yahboom_joy_R2 >/tmp/yahboom_joy_R2.log 2>&1 &
sleep 2
grep -n 'axes\[0\]' ~/yahboomcar_ros2_ws/yahboomcar_ws/install/yahboomcar_ctrl/lib/python3.10/site-packages/yahboomcar_ctrl/yahboom_joy_R2.py | head -1
ps aux | grep -E 'joy_node|yahboom_joy_R2' | grep -v grep
REMOTE

echo "Done. Press joystick button '2' to toggle Joy ON, use LEFT stick only."