#!/bin/bash
# Stop patrol/mapping stack and close the R2 Patrol terminal.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/stop_robot_stack.sh"
# Inner start_patrol trap will clean up; this lets the gnome-terminal exit.
pkill -f '/home/jetson/ROSMASTER-R2-fork/scripts/start_patrol.sh' 2>/dev/null || true
exit 0
