#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
. /opt/ros/jazzy/setup.bash
. install/setup.bash
ros2 launch navigation_bringup full_system.launch.py "$@"
