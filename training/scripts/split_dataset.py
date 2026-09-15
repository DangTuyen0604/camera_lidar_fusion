#!/usr/bin/env python3
"""Create deterministic train/validation file lists for a YOLO dataset."""

import argparse
from pathlib import Path
import random


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--validation-ratio', type=float, default=0.2)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if not 0.0 < args.validation_ratio < 1.0:
        parser.error('--validation-ratio must be between 0 and 1')
    images = sorted(
        path for path in args.images.iterdir()
        if path.suffix.lower() in {'.jpg', '.jpeg', '.png'}
    )
    random.Random(args.seed).shuffle(images)
    split = round(len(images) * (1.0 - args.validation_ratio))
    args.output.mkdir(parents=True, exist_ok=True)
    for name, items in [('train.txt', images[:split]), ('val.txt', images[split:])]:
        content = ''.join(f'{path.resolve()}\n' for path in items)
        (args.output / name).write_text(content, encoding='utf-8')


if __name__ == '__main__':
    main()
