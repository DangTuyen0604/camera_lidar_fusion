#!/usr/bin/env python3
"""Add reproducible Gaussian noise and dropout to KITTI point clouds."""

import argparse
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--standard-deviation', type=float, default=0.02)
    parser.add_argument('--dropout-probability', type=float, default=0.0)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    if args.standard_deviation < 0.0:
        parser.error('--standard-deviation must be non-negative')
    if not 0.0 <= args.dropout_probability < 1.0:
        parser.error('--dropout-probability must be in [0, 1)')

    points = np.fromfile(args.input, dtype='<f4')
    if points.size % 4:
        raise SystemExit('KITTI point cloud must contain groups of four float32 values')
    points = points.reshape(-1, 4)
    generator = np.random.default_rng(args.seed)
    keep = generator.random(points.shape[0]) >= args.dropout_probability
    output = points[keep].copy()
    output[:, :3] += generator.normal(
        0.0, args.standard_deviation, size=output[:, :3].shape
    )
    displacement = output[:, :3] - points[keep, :3]
    rms_displacement = float(np.sqrt(np.mean(displacement * displacement))) \
        if len(output) else 0.0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    output.astype('<f4').tofile(args.output)
    print(
        f'Wrote {len(output)} of {len(points)} points to {args.output}; '
        f'RMS coordinate displacement={rms_displacement:.6f} m; '
        f'dropout={(1.0 - len(output) / len(points)):.3%}'
    )


if __name__ == '__main__':
    main()
