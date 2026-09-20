#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/camera_lidar_fusion_audit_logs}"
mkdir -p "$ROS_LOG_DIR"
. /opt/ros/jazzy/setup.bash
[[ ! -f install/setup.bash ]] || . install/setup.bash

python3 tools/audit_repository.py
python3 tools/audit_ros_contracts.py
python3 -m compileall -q benchmarks scripts tools src

mapfile -t shell_files < <(find scripts docker tools -type f -name '*.sh' -print | sort)
for file in "${shell_files[@]}"; do bash -n "$file"; done
if ! command -v shellcheck >/dev/null; then
  echo 'shellcheck is required; run scripts/install_dependencies.sh' >&2
  exit 1
fi
shellcheck "${shell_files[@]}"

mapfile -t yaml_files < <(find . -type f \( -name '*.yaml' -o -name '*.yml' \) \
  -not -path './build/*' -not -path './install/*' -not -path './log/*' \
  -not -path './.git/*' | sort)
python3 - "${yaml_files[@]}" <<'PY'
from pathlib import Path
import sys
import yaml
for name in sys.argv[1:]:
    assert yaml.safe_load(Path(name).read_text(encoding='utf-8')) is not None, name
PY

while IFS= read -r file; do xmllint --noout "$file"; done < <(
  find src -type f \( -name '*.xml' -o -name 'package.xml' \) | sort)
while IFS= read -r file; do xacro "$file" >/dev/null; done < <(
  find src -type f -name '*.xacro' | sort)

ament_flake8 benchmarks tools src
ament_pep257 benchmarks tools src

mapfile -t cpp_files < <(find src -type f \( -name '*.cpp' -o -name '*.hpp' \) | sort)
ament_uncrustify "${cpp_files[@]}"

rosdep check --from-paths src --ignore-src --rosdistro jazzy

if [[ "${1:-}" == '--full' ]]; then
  ./scripts/run_tests.sh
fi

echo 'Gate 7.1 audit checks completed successfully.'
