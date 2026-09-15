#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_ros_logs}"
mkdir -p "$ROS_LOG_DIR"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
. /opt/ros/jazzy/setup.bash
[[ ! -f install/setup.bash ]] || . install/setup.bash
set -u

test_status=0
colcon test --event-handlers console_direct+ "$@" || test_status=$?
result_status=0
colcon test-result --verbose || result_status=$?

if (( test_status != 0 )); then
  exit "$test_status"
fi
exit "$result_status"
