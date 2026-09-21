#!/usr/bin/env bash
set -eo pipefail
set +u

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_benchmark_logs}"
mkdir -p "$ROS_LOG_DIR"
. /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo 'Workspace is not built. Run scripts/build_workspace.sh first.' >&2
  exit 1
fi
. install/setup.bash
set -u
python3 benchmarks/run_benchmark.py "$@"
python3 benchmarks/plot_results.py
