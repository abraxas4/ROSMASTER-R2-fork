#!/bin/bash
# Persistent desktop battery readout. Safe to run at login.
export DISPLAY="${DISPLAY:-:0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LIBRARY_WS_HOST="${LIBRARY_WS_HOST:-$HOME/yahboomcar_ros2_ws/software/library_ws}"
export NATIVE_WS_HOST="${NATIVE_WS_HOST:-$HOME/yahboomcar_ros2_ws/yahboomcar_ws}"
set +u
# shellcheck source=native_ros_setup.bash
source "$SCRIPT_DIR/native_ros_setup.bash" 2>/dev/null || true
set -u
exec python3 "$SCRIPT_DIR/battery_osd.py"
