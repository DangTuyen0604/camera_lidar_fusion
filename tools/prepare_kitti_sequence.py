#!/usr/bin/env python3
"""Validate and prepare a size-limited KITTI sequence."""

import argparse
from pathlib import Path
import shutil


def copy_group(source, destination, names, use_symlinks):
    destination.mkdir(parents=True, exist_ok=True)
    for name in names:
        target = destination / name
        if use_symlinks:
            target.symlink_to((source / name).resolve())
        else:
            shutil.copy2(source / name, target)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--copy', action='store_true', help='copy instead of symlink')
    args = parser.parse_args()
    image_dir = args.source / 'image_02' / 'data'
    cloud_dir = args.source / 'velodyne_points' / 'data'
    images = sorted(path.name for path in image_dir.glob('*.png'))
    clouds = sorted(path.name for path in cloud_dir.glob('*.bin'))
    image_ids = [Path(name).stem for name in images]
    cloud_ids = [Path(name).stem for name in clouds]
    if image_ids != cloud_ids:
        raise SystemExit('Image and point-cloud frame IDs do not match')
    if args.limit < 0:
        parser.error('--limit must be non-negative')
    if args.limit:
        images = images[:args.limit]
        clouds = clouds[:args.limit]
    if args.destination.exists() and any(args.destination.iterdir()):
        raise SystemExit('Destination must be empty')
    copy_group(
        image_dir, args.destination / 'image_02' / 'data', images, not args.copy
    )
    copy_group(
        cloud_dir, args.destination / 'velodyne_points' / 'data', clouds,
        not args.copy
    )
    for sensor in ('image_02', 'velodyne_points'):
        timestamps = (args.source / sensor / 'timestamps.txt').read_text(
            encoding='utf-8'
        ).splitlines()
        count = len(images)
        target = args.destination / sensor / 'timestamps.txt'
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('\n'.join(timestamps[:count]) + '\n', encoding='utf-8')
    print(f'Prepared {len(images)} aligned frames in {args.destination}')


if __name__ == '__main__':
    main()
