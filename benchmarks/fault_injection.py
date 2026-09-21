"""Deterministic data transformations used by fault scenarios and tests."""

import math

import numpy as np


def inject_timestamp_delay_ns(original_stamp_ns, delay_ms):
    """Return a nanosecond timestamp delayed by the requested milliseconds."""
    if not math.isfinite(delay_ms):
        raise ValueError('delay_ms must be finite')
    return int(original_stamp_ns) + int(round(delay_ms * 1_000_000.0))


def inject_point_noise(points, stddev, seed):
    """Add seeded Gaussian noise to XYZ while preserving extra point fields."""
    values = np.asarray(points)
    if values.ndim != 2 or values.shape[1] < 3:
        raise ValueError('points must have shape (N, >=3)')
    if not math.isfinite(stddev) or stddev < 0.0:
        raise ValueError('stddev must be finite and non-negative')
    output = values.astype(np.result_type(values.dtype, np.float32), copy=True)
    generator = np.random.default_rng(int(seed))
    output[:, :3] += generator.normal(0.0, stddev, size=(len(values), 3))
    return output


def reduce_cloud_density(points, density, seed):
    """Select a deterministic fraction of a point cloud without replacement."""
    values = np.asarray(points)
    if values.ndim != 2:
        raise ValueError('points must be a two-dimensional array')
    if not math.isfinite(density) or not 0.0 < density <= 1.0:
        raise ValueError('density must be in (0, 1]')
    target_count = int(round(len(values) * density))
    if not len(values) or target_count == len(values):
        return values.copy()
    generator = np.random.default_rng(int(seed))
    indices = generator.choice(len(values), size=target_count, replace=False)
    return values[indices].copy()


def inject_calibration_drift(transform, translation_m, rotation_deg):
    """Apply deterministic X translation and camera-Z rotation drift."""
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('transform must have shape (4, 4)')
    if not np.all(np.isfinite(matrix)):
        raise ValueError('transform must be finite')
    if not math.isfinite(translation_m) or not math.isfinite(rotation_deg):
        raise ValueError('calibration drift must be finite')
    angle = math.radians(rotation_deg)
    drift = np.eye(4, dtype=np.float64)
    drift[0, 3] = translation_m
    drift[:3, :3] = [
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ]
    return drift @ matrix


def project_point(point_xyz, lidar_to_camera, camera_matrix):
    """Project one LiDAR point and reject points behind the camera."""
    point = np.asarray(point_xyz, dtype=np.float64)
    transform = np.asarray(lidar_to_camera, dtype=np.float64)
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    if point.shape != (3,) or transform.shape != (4, 4) or intrinsics.shape != (3, 3):
        raise ValueError('expected point (3,), transform (4,4), intrinsics (3,3)')
    camera_point = transform @ np.append(point, 1.0)
    if camera_point[2] <= 0.0:
        raise ValueError('point must be in front of the camera')
    homogeneous = intrinsics @ camera_point[:3]
    return homogeneous[:2] / homogeneous[2]
