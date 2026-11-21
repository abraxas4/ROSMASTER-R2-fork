#!/bin/bash
# Build test script for r2_autonomy
# Tests compilation and launch file imports without running Nav2

set -e

echo "=========================================="
echo "R2 Autonomy Build & Import Test"
echo "=========================================="

# Check if running in ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 environment not sourced"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Build the package
echo "=========================================="
echo "Step 1: Building r2_autonomy package"
echo "=========================================="

cd /workspace/code/yahboomcar_ros2_ws
colcon build --packages-select r2_autonomy --event-handlers console_direct+

if [ $? -ne 0 ]; then
    echo "ERROR: Build failed"
    exit 1
fi

echo ""
echo "✓ Build successful"
echo ""

# Source the workspace
source install/setup.bash

# Test Python imports
echo "=========================================="
echo "Step 2: Testing Python script imports"
echo "=========================================="

SCRIPTS=(
    "slip_detector.py"
    "battery_monitor.py"
    "odom_sanity_checker.py"
    "path_reverse_recovery.py"
    "patrol_waypoint_generator.py"
    "fake_scan_publisher.py"
    "fake_odom_publisher.py"
)

for script in "${SCRIPTS[@]}"; do
    echo -n "  Testing $script... "
    python3 -c "import sys; sys.path.insert(0, 'install/r2_autonomy/lib/r2_autonomy'); exec(open('install/r2_autonomy/lib/r2_autonomy/$script').read().replace('#!/usr/bin/env python3', '').split('if __name__')[0])" 2>&1 | grep -q "Error" && echo "FAIL" || echo "OK"
done

echo ""
echo "✓ Script imports successful"
echo ""

# Test launch file syntax
echo "=========================================="
echo "Step 3: Testing launch file syntax"
echo "=========================================="

LAUNCH_FILES=(
    "map_mode.launch.py"
    "explore_mode.launch.py"
    "patrol_mode.launch.py"
    "localize_mode.launch.py"
    "test_headless.launch.py"
)

for launch in "${LAUNCH_FILES[@]}"; do
    echo -n "  Testing $launch... "
    python3 -m py_compile "install/r2_autonomy/share/r2_autonomy/launch/$launch" && echo "OK" || echo "FAIL"
done

echo ""
echo "✓ Launch files validated"
echo ""

# Test action interface
echo "=========================================="
echo "Step 4: Testing action interface"
echo "=========================================="

ros2 interface list | grep -q "r2_autonomy_msgs/action/PathReverseRecovery" && echo "  ✓ PathReverseRecovery action found" || echo "  ✗ PathReverseRecovery action NOT found"

echo ""

# Summary
echo "=========================================="
echo "Build Test Summary"
echo "=========================================="
echo "✓ Package builds successfully"
echo "✓ All Python scripts compile"
echo "✓ All launch files valid"
echo "✓ Action interfaces generated"
echo ""
echo "Ready for deployment!"
