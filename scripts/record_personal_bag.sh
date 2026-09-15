#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

. /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built. Run scripts/build_workspace.sh first." >&2
  exit 1
fi
. install/setup.bash
set -u

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_output="${1:-bags/kitti_projection_${timestamp}}"

if [[ -e "$bag_output" ]]; then
  echo "Bag output already exists: $bag_output" >&2
  exit 1
fi

mkdir -p "$(dirname "$bag_output")"
echo "Recording to $bag_output. Press Ctrl+C to stop cleanly."

ros2 bag record \
  --storage mcap \
  --max-cache-size 536870912 \
  --disable-keyboard-controls \
  --output "$bag_output" \
  --topics \
  /kitti/camera/image_raw \
  /kitti/camera/camera_info \
  /kitti/velodyne/points \
  /kitti/camera/lidar_overlay \
  /fusion/synced/image \
  /fusion/synced/camera_info \
  /fusion/synced/points \
  /fusion/sync_status \
  /detections_2d \
  /fusion/detections_3d \
  /fusion/annotated_image \
  /fusion/object_markers \
  /tf_static
