#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
. /opt/ros/jazzy/setup.bash
. install/setup.bash
ros2 launch fusion_bringup projection_demo.launch.py "$@"
