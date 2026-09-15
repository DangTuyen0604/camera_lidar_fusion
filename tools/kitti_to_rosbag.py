#!/usr/bin/env python3
"""Record a bounded KITTI playback into a rosbag using project launch files."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


TOPICS = [
    '/kitti/camera/image_raw',
    '/kitti/camera/camera_info',
    '/kitti/velodyne/points',
    '/kitti/camera/lidar_overlay',
    '/tf_static',
]


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset_root', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--publish-rate', type=float, default=10.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.publish_rate <= 0:
        parser.error('Duration and publish rate must be positive')
    if args.output.exists():
        raise SystemExit(f'Output already exists: {args.output}')
    if not os.environ.get('AMENT_PREFIX_PATH'):
        raise SystemExit('Source ROS and install/setup.bash before running')

    recorder = subprocess.Popen([
        'ros2', 'bag', 'record', '-o', str(args.output), *TOPICS,
    ])
    player = None
    try:
        time.sleep(2.0)
        player = subprocess.Popen([
            'ros2', 'launch', 'fusion_bringup', 'projection_demo.launch.py',
            f'dataset_root:={args.dataset_root.resolve()}',
            f'publish_rate:={args.publish_rate}',
            'loop:=false', 'use_rviz:=false',
        ])
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and player.poll() is None:
            time.sleep(0.2)
    finally:
        if player is not None:
            stop(player)
        stop(recorder)
    if recorder.returncode not in (0, -signal.SIGINT):
        raise SystemExit(recorder.returncode)
    print(f'Bag written to {args.output}')


if __name__ == '__main__':
    main()
