#!/usr/bin/env python3
"""Create deterministic train/validation file lists for a YOLO dataset."""

import argparse
from pathlib import Path
import random
import shutil


def place_file(source, destination, copy_files):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if copy_files:
        shutil.copy2(source, destination)
    else:
        destination.symlink_to(source.resolve())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--validation-ratio', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--labels', type=Path)
    parser.add_argument('--copy', action='store_true')
    args = parser.parse_args()
    if not 0.0 < args.validation_ratio < 1.0:
        parser.error('--validation-ratio must be between 0 and 1')
    images = sorted(
        path for path in args.images.iterdir()
        if path.suffix.lower() in {'.jpg', '.jpeg', '.png'}
    )
    if len(images) < 2:
        raise SystemExit('At least two images are required for a split')
    random.Random(args.seed).shuffle(images)
    split = round(len(images) * (1.0 - args.validation_ratio))
    args.output.mkdir(parents=True, exist_ok=True)
    for name, items in [('train.txt', images[:split]), ('val.txt', images[split:])]:
        content = ''.join(f'{path.resolve()}\n' for path in items)
        (args.output / name).write_text(content, encoding='utf-8')
    if args.labels is not None:
        if not args.labels.is_dir():
            raise SystemExit(f'Label directory not found: {args.labels}')
        for split_name, items in (
            ('train', images[:split]),
            ('val', images[split:]),
        ):
            for image in items:
                label = args.labels / f'{image.stem}.txt'
                if not label.is_file():
                    raise SystemExit(f'Missing label for {image.name}: {label}')
                place_file(
                    image,
                    args.output / 'images' / split_name / image.name,
                    args.copy,
                )
                place_file(
                    label,
                    args.output / 'labels' / split_name / label.name,
                    args.copy,
                )
    print(
        f'Split {len(images)} images: train={split}, '
        f'val={len(images) - split}, seed={args.seed}'
    )


if __name__ == '__main__':
    main()
