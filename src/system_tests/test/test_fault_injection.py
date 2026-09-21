"""Behavioral tests for deterministic perception fault injection."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[3] / 'benchmarks' / 'fault_injection.py'
)
SPEC = importlib.util.spec_from_file_location('fault_injection', MODULE_PATH)
faults = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(faults)
SEED = 12345


def test_timestamp_delay_changes_stamp_exactly():
    original = 10_000_000_000
    delayed = faults.inject_timestamp_delay_ns(original, delay_ms=80.0)
    assert delayed - original == 80_000_000


def test_point_noise_is_seeded_and_preserves_intensity():
    points = np.array([
        [1.0, 2.0, 3.0, 0.25],
        [4.0, 5.0, 6.0, 0.75],
    ], dtype=np.float32)
    first = faults.inject_point_noise(points, stddev=0.05, seed=SEED)
    second = faults.inject_point_noise(points, stddev=0.05, seed=SEED)
    assert np.array_equal(first, second)
    assert not np.array_equal(first[:, :3], points[:, :3])
    assert np.array_equal(first[:, 3], points[:, 3])
    expected_xyz = np.array([
        [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    assert np.array_equal(points[:, :3], expected_xyz)


def test_cloud_density_is_seeded_and_reduces_count():
    points = np.arange(400, dtype=np.float32).reshape(100, 4)
    first = faults.reduce_cloud_density(points, density=0.25, seed=SEED)
    second = faults.reduce_cloud_density(points, density=0.25, seed=SEED)
    assert len(first) == 25
    assert np.array_equal(first, second)


def test_calibration_drift_changes_projection():
    point = np.array([2.0, 1.0, 10.0])
    transform = np.eye(4)
    intrinsics = np.array([
        [700.0, 0.0, 600.0],
        [0.0, 700.0, 180.0],
        [0.0, 0.0, 1.0],
    ])
    clean = faults.project_point(point, transform, intrinsics)
    drifted_transform = faults.inject_calibration_drift(
        transform, translation_m=0.05, rotation_deg=3.0)
    drifted = faults.project_point(point, drifted_transform, intrinsics)
    assert np.linalg.norm(drifted - clean) > 1.0


def test_fault_removal_recovers_baseline():
    points = np.arange(60, dtype=np.float32).reshape(20, 3)
    baseline = faults.inject_point_noise(points, stddev=0.0, seed=SEED)
    faulty = faults.inject_point_noise(points, stddev=0.10, seed=SEED)
    recovered = faults.inject_point_noise(points, stddev=0.0, seed=SEED)
    assert not np.array_equal(faulty, baseline)
    np.testing.assert_allclose(recovered, baseline, rtol=1e-6, atol=1e-7)


@pytest.mark.parametrize('density', [0.0, -0.1, 1.1, float('nan')])
def test_invalid_density_is_rejected(density):
    with pytest.raises(ValueError, match='density'):
        faults.reduce_cloud_density(np.ones((4, 3)), density, SEED)
