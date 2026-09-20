#!/usr/bin/env bash
set -euo pipefail

compose=(docker compose -f docker/compose.yaml --profile runtime-cpu)
cleanup() {
  "${compose[@]}" down --remove-orphans
}
trap cleanup EXIT INT TERM

"${compose[@]}" build simulation perception navigation mission_manager
"${compose[@]}" up -d simulation perception navigation mission_manager
timeout 90s bash -c '
  until docker compose -f docker/compose.yaml exec -T perception \
    ros2 topic list | grep -q /fusion/detections_3d; do sleep 2; done
'
"${compose[@]}" ps --status running
