#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
compose=(docker compose -f docker/compose.yaml --profile runtime-cpu)
cleanup() { "${compose[@]}" down --remove-orphans; }
trap cleanup EXIT INT TERM

[[ -d data/kitti/2011_09_26 ]] || { echo 'KITTI dataset is missing' >&2; exit 1; }
[[ -f models/yolov8n-opencv.onnx ]] || { echo 'ONNX model is missing' >&2; exit 1; }
"${compose[@]}" build
docker compose -f docker/compose.yaml --profile training build training
"${compose[@]}" up -d simulation perception navigation mission_manager

for topic in /kitti/camera/lidar_overlay /fusion/detections_3d /mission/state; do
  timeout 120s bash -c "until docker compose -f docker/compose.yaml exec -T perception \
    ros2 topic list | grep -Fxq '$topic'; do sleep 2; done"
done
"${compose[@]}" ps --status running
echo 'Docker runtime projection, perception, warehouse and mission smoke checks passed.'
