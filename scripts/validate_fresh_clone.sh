#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remote="${FRESH_CLONE_REMOTE:-$(git -C "$project_root" remote get-url origin)}"
branch="${FRESH_CLONE_BRANCH:-$(git -C "$project_root" branch --show-current)}"
validation_root="$(mktemp -d -t camera-lidar-fresh-clone-XXXXXX)"
trap 'rm -rf "$validation_root"' EXIT INT TERM

git clone --branch "$branch" --single-branch "$remote" "$validation_root/repository"
cd "$validation_root/repository"
./scripts/install_dependencies.sh
./scripts/build_workspace.sh
./tools/download_kitti.sh

smoke_launch() {
  local name="$1"
  shift
  set +e
  timeout --signal=INT --kill-after=10s 30s "$@"
  local status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "$name failed with exit code $status" >&2
    return "$status"
  fi
}

smoke_launch projection ./scripts/run_projection_demo.sh use_rviz:=false
smoke_launch perception ./scripts/run_perception_demo.sh use_rviz:=false
smoke_launch calibration ./scripts/run_calibration_demo.sh use_rviz:=false
smoke_launch navigation ./scripts/run_navigation_demo.sh use_sim_time:=true
smoke_launch warehouse ./scripts/run_warehouse_demo.sh gui:=false
./scripts/run_tests.sh
./scripts/run_benchmarks.sh
./docker/validate_runtime.sh

echo "Gate 7.2 passed from fresh clone: $remote ($branch)"
