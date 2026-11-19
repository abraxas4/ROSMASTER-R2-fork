#!/bin/bash
set -e

echo "Building r2_autonomy test environment..."

cd "$(dirname "$0")"
docker build -t r2_autonomy_test:humble -f Dockerfile.test .

echo ""
echo "Test container built. Run with:"
echo "  docker run --rm -it -v $(pwd)/../../..:/workspace/code/yahboomcar_ros2_ws r2_autonomy_test:humble"
