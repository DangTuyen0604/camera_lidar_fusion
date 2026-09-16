#!/usr/bin/env python3
"""Validate and prepare a size-limited KITTI sequence."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


def read_timestamps(path):
    if not path.is_file():
        raise RuntimeError(f'Missing timestamp file: {path}')
    lines = [line.strip() for line in path.read_text(
        encoding='utf-8'
    ).splitlines() if line.strip()]
    parsed = []
    for line in lines:
        try:
            date_part, fractional_part = line.split('.', 1)
            timestamp = datetime.strptime(
                date_part,
                '%Y-%m-%d %H:%M:%S',
            ).replace(tzinfo=timezone.utc)
            if not fractional_part.isdigit() or len(fractional_part) > 9:
                raise ValueError
            parsed.append(
                int(timestamp.timestamp()) * 1_000_000_000
                + int(fractional_part.ljust(9, '0'))
            )
        except ValueError as error:
            raise RuntimeError(f'Invalid KITTI timestamp in {path}: {line}') from error
    if any(current <= previous for previous, current in zip(parsed, parsed[1:])):
        raise RuntimeError(f'Timestamps must be strictly increasing: {path}')
    return lines


def validate_frames(image_dir, cloud_dir, image_names, cloud_names):
    if not image_names or not cloud_names:
        raise RuntimeError('KITTI sequence contains no image/point-cloud frames')
    image_ids = [Path(name).stem for name in image_names]
    cloud_ids = [Path(name).stem for name in cloud_names]
    if image_ids != cloud_ids:
        raise RuntimeError('Image and point-cloud frame IDs do not match')

    expected_shape = None
    for image_name, cloud_name in zip(image_names, cloud_names):
        image = cv2.imread(str(image_dir / image_name), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'Cannot decode KITTI image: {image_name}')
        if expected_shape is None:
            expected_shape = image.shape
        elif image.shape != expected_shape:
            raise RuntimeError(
                f'Inconsistent image shape: {image_name}: {image.shape} '
                f'!= {expected_shape}'
            )
        raw_points = np.fromfile(cloud_dir / cloud_name, dtype='<f4')
        if raw_points.size == 0 or raw_points.size % 4 != 0:
            raise RuntimeError(f'Invalid KITTI point cloud: {cloud_name}')
        if not np.isfinite(raw_points).all():
            raise RuntimeError(f'Non-finite KITTI point cloud: {cloud_name}')
    return expected_shape


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
    parser.add_argument('--expected-frames', type=int, default=0)
    parser.add_argument('--copy', action='store_true', help='copy instead of symlink')
    args = parser.parse_args()
    image_dir = args.source / 'image_02' / 'data'
    cloud_dir = args.source / 'velodyne_points' / 'data'
    images = sorted(path.name for path in image_dir.glob('*.png'))
    clouds = sorted(path.name for path in cloud_dir.glob('*.bin'))
    try:
        image_timestamps = read_timestamps(
            args.source / 'image_02' / 'timestamps.txt'
        )
        cloud_timestamps = read_timestamps(
            args.source / 'velodyne_points' / 'timestamps.txt'
        )
        image_shape = validate_frames(
            image_dir,
            cloud_dir,
            images,
            clouds,
        )
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    if len(image_timestamps) != len(images):
        raise SystemExit(
            'Image timestamp count does not match image count: '
            f'{len(image_timestamps)} != {len(images)}'
        )
    if len(cloud_timestamps) != len(clouds):
        raise SystemExit(
            'Point-cloud timestamp count does not match cloud count: '
            f'{len(cloud_timestamps)} != {len(clouds)}'
        )
    if args.expected_frames and len(images) != args.expected_frames:
        raise SystemExit(
            f'Expected {args.expected_frames} frames, found {len(images)}'
        )
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
    for sensor, timestamps in (
        ('image_02', image_timestamps),
        ('velodyne_points', cloud_timestamps),
    ):
        count = len(images)
        target = args.destination / sensor / 'timestamps.txt'
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('\n'.join(timestamps[:count]) + '\n', encoding='utf-8')
    manifest = {
        'source': str(args.source.resolve()),
        'frame_count': len(images),
        'image_height': image_shape[0],
        'image_width': image_shape[1],
        'storage': 'copy' if args.copy else 'symlink',
        'first_frame': Path(images[0]).stem,
        'last_frame': Path(images[-1]).stem,
    }
    (args.destination / 'manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n',
        encoding='utf-8',
    )
    print(f'Prepared {len(images)} aligned frames in {args.destination}')


if __name__ == '__main__':
    main()
