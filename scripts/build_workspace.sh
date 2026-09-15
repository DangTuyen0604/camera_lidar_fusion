#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
. /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install --event-handlers console_direct+ "$@"

echo "Build complete. Run: source install/setup.bash"
