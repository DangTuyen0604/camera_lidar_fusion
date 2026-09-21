#!/usr/bin/env bash
set -eo pipefail
set +u

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_release_demo_logs}"
mkdir -p "$ROS_LOG_DIR"

if [[ ! -x "$project_root/.venv/bin/python" ]]; then
  echo 'Python environment is missing.' >&2
  echo 'Run once: ./scripts/install_dependencies.sh' >&2
  exit 1
fi
venv_site="$("$project_root/.venv/bin/python" -c \
  'import site; print(site.getsitepackages()[0])')"
export PYTHONPATH="$venv_site${PYTHONPATH:+:$PYTHONPATH}"

. /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo 'Workspace is not built. Run scripts/build_workspace.sh first.' >&2
  exit 1
fi
. install/setup.bash
if ! python3 -c 'import onnxruntime' 2>/dev/null; then
  echo 'ONNX Runtime is missing from .venv.' >&2
  echo 'Run once: .venv/bin/python -m pip install "numpy<2" "onnxruntime>=1.20,<2"' >&2
  exit 1
fi
set -u
ros2 launch fusion_bringup bringup_sim.launch.py "$@"
