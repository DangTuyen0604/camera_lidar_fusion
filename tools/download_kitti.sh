#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dataset_dir="$project_root/data/kitti"
temporary_dir="$(mktemp -d)"
trap 'rm -rf "$temporary_dir"' EXIT

calibration_url="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip"
sequence_url="https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/2011_09_26_drive_0005_sync.zip"

mkdir -p "$dataset_dir"
curl -fL "$calibration_url" -o "$temporary_dir/calibration.zip"
curl -fL "$sequence_url" -o "$temporary_dir/sequence.zip"
unzip -q "$temporary_dir/calibration.zip" -d "$dataset_dir"
unzip -q "$temporary_dir/sequence.zip" -d "$dataset_dir"

image_count="$(find "$dataset_dir/2011_09_26/2011_09_26_drive_0005_sync/image_02/data" -maxdepth 1 -name '*.png' -type f | wc -l)"
pointcloud_count="$(find "$dataset_dir/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data" -maxdepth 1 -name '*.bin' -type f | wc -l)"

if [[ "$image_count" -ne 154 || "$pointcloud_count" -ne 154 ]]; then
  echo "Unexpected KITTI frame count: images=$image_count pointclouds=$pointcloud_count" >&2
  exit 1
fi

echo "KITTI drive 0005 ready: 154 synchronized frames in $dataset_dir"
