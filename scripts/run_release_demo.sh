#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_release_demo_logs}"
mkdir -p "$ROS_LOG_DIR"
. /opt/ros/jazzy/setup.bash
. install/setup.bash
ros2 launch system_tests day6_benchmark.launch.py gui:=true use_rviz:=true "$@"
