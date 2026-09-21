#!/usr/bin/env bash
set -eo pipefail
set +u

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_ros_logs}"
mkdir -p "$ROS_LOG_DIR"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
. /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo 'Workspace is not built. Run scripts/build_workspace.sh first.' >&2
  exit 1
fi
. install/setup.bash

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_DISABLE_DAEMON=1
domain_id="${ROS_TEST_DOMAIN_ID:-$((20 + $$ % 80))}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 0 || domain_id > 101)); then
  echo "Invalid ROS test domain: $domain_id (expected 0..101)" >&2
  exit 2
fi
export ROS_DOMAIN_ID="$domain_id"
set -u

echo "ROS test isolation: domain=$ROS_DOMAIN_ID discovery=$ROS_AUTOMATIC_DISCOVERY_RANGE"

# Do not let XML files from an earlier, broader test run contaminate a
# package- or regex-selected rerun.
colcon test-result --delete-yes >/dev/null 2>&1 || true

test_status=0
colcon test --event-handlers console_direct+ "$@" || test_status=$?
result_status=0
colcon test-result --verbose || result_status=$?

if (( test_status != 0 )); then
  exit "$test_status"
fi
exit "$result_status"
