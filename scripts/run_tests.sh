#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_ros_logs}"
mkdir -p "$ROS_LOG_DIR"
. /opt/ros/jazzy/setup.bash
[[ ! -f install/setup.bash ]] || . install/setup.bash
colcon test --event-handlers console_direct+ "$@"
colcon test-result --verbose
