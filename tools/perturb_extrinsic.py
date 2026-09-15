#!/usr/bin/env python3
"""Create a perturbed LiDAR-camera extrinsic YAML for drift tests."""

import argparse
import math
from pathlib import Path

import numpy as np
import yaml


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def euler_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = (
        math.radians(value) / 2.0
        for value in (roll_deg, pitch_deg, yaw_deg)
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def perturb_document(document, translation, rotation_deg):
    result = dict(document)
    result['translation'] = dict(document['translation'])
    for axis, delta in zip(('x', 'y', 'z'), translation):
        result['translation'][axis] = float(result['translation'][axis]) + delta
    source = document['rotation']
    base = np.array([source['x'], source['y'], source['z'], source['w']])
    updated = quaternion_multiply(euler_quaternion(*rotation_deg), base)
    updated /= np.linalg.norm(updated)
    result['rotation'] = {
        axis: float(value)
        for axis, value in zip(('x', 'y', 'z', 'w'), updated)
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--translation', nargs=3, type=float, default=(0, 0, 0))
    parser.add_argument('--rotation-deg', nargs=3, type=float, default=(0, 0, 0))
    args = parser.parse_args()
    with args.input.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    result = perturb_document(document, args.translation, args.rotation_deg)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(result, stream, sort_keys=False)
    print(f'Wrote perturbed extrinsic to {args.output}')


if __name__ == '__main__':
    main()
