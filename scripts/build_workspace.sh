#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

. /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install

echo "Build complete. Run: source install/setup.bash"
