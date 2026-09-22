#!/usr/bin/env python3
"""Measure the five implemented deterministic core faults."""

import argparse
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np


def load_faults(root):
    """Load the existing fault transformations from this repository."""
    path = root / 'benchmarks' / 'fault_injection.py'
    spec = importlib.util.spec_from_file_location('fault_injection', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=12014)
    parser.add_argument(
        '--canonical-results', type=Path,
        default=Path('benchmarks/canonical_demo_results.json'))
    parser.add_argument(
        '--output', type=Path,
        default=Path('experiments/gate14_core_faults.json'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    faults = load_faults(root)

    sync_test = root / 'build' / 'perception_core' / 'test_timestamp_sync'
    completed = subprocess.run(
        [str(sync_test),
         '--gtest_filter=TimestampSync.DetectsDelayAboveFiftyMilliseconds'],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False)
    if completed.returncode != 0:
        raise RuntimeError(completed.stdout)
    original_ns = 1_000_000_000
    delayed_ns = faults.inject_timestamp_delay_ns(original_ns, 80.0)

    point = np.array([2.0, 1.0, 10.0])
    transform = np.eye(4)
    intrinsics = np.array([
        [700.0, 0.0, 600.0],
        [0.0, 700.0, 180.0],
        [0.0, 0.0, 1.0],
    ])
    clean_pixel = faults.project_point(point, transform, intrinsics)
    drifted_transform = faults.inject_calibration_drift(
        transform, translation_m=0.05, rotation_deg=3.0)
    drifted_pixel = faults.project_point(point, drifted_transform, intrinsics)

    points = np.column_stack((
        np.linspace(1.0, 4.0, 100),
        np.linspace(-0.5, 0.5, 100),
        np.linspace(2.0, 5.0, 100),
        np.ones(100)))
    noisy = faults.inject_point_noise(points, stddev=0.10, seed=args.seed)
    rms_noise = float(np.sqrt(np.mean(
        (noisy[:, :3] - points[:, :3]) ** 2)))
    reduced = faults.reduce_cloud_density(points, density=0.25, seed=args.seed)
    repeated = faults.reduce_cloud_density(points, density=0.25, seed=args.seed)

    canonical = json.loads(args.canonical_results.read_text(encoding='utf-8'))
    expiry_values = [run['obstacle_expiry_s'] for run in canonical['runs']]
    empty_heartbeats = [
        run['evidence']['empty_bridge_heartbeats_after_removal']
        for run in canonical['runs']]
    result = {
        'seed': args.seed,
        'timestamp_delay': {
            'injected_ms': (delayed_ns - original_ns) / 1.0e6,
            'tolerance_ms': 50.0,
            'observable_status': 'DELAYED',
            'healthy': False,
            'drop_verified_by_core_test': True,
        },
        'calibration_perturbation': {
            'translation_m': 0.05,
            'rotation_deg': 3.0,
            'projection_error_increase_px': round(float(np.linalg.norm(
                drifted_pixel - clean_pixel)), 6),
        },
        'pointcloud_noise': {
            'stddev_m': 0.10,
            'measured_xyz_rms_change_m': round(rms_noise, 6),
        },
        'density_reduction': {
            'input_points': len(points),
            'output_points': len(reduced),
            'retained_fraction': len(reduced) / len(points),
            'deterministic_repeat_equal': bool(np.array_equal(reduced, repeated)),
        },
        'lost_detection': {
            'canonical_runs': len(expiry_values),
            'obstacle_expiry_mean_s': round(float(np.mean(expiry_values)), 6),
            'minimum_empty_heartbeats': min(empty_heartbeats),
            'obstacle_expired_in_every_run': all(value >= 2 for value in empty_heartbeats),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
