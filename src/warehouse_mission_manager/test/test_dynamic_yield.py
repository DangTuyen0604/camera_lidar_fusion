import math

import numpy as np
import pytest

from warehouse_mission_manager.motion_tracker import MotionTracker, TrackerConfig, clusters
from warehouse_mission_manager.yield_gate import (
    CLEAR, TIMED_OUT, YIELDING, YieldConfig, YieldGate)


def blob(x, y, n=6, radius=0.15):
    """Scan-ordered points on the near side of a round object at (x, y)."""
    angles = np.linspace(-math.pi / 2, math.pi / 2, n) + math.pi
    return np.c_[x + radius * np.cos(angles), y + radius * np.sin(angles)]


def test_clusters_split_objects_and_reject_walls():
    wall = np.c_[np.linspace(-3, 3, 60), np.full(60, 4.0)]
    points = np.vstack([blob(2.0, 0.0), blob(2.0, 1.5), wall])
    centroids = clusters(points, TrackerConfig())
    assert len(centroids) == 2  # the 6 m wall is not an object
    assert {round(c[1], 1) for c in centroids} == {0.0, 1.5}


def walk(tracker, start, velocity, steps, dt=0.1, t0=0.0):
    for k in range(steps):
        position = np.asarray(start) + np.asarray(velocity) * k * dt
        tracker.update(t0 + k * dt, clusters(blob(*position), tracker.config))
    return t0 + (steps - 1) * dt


@pytest.mark.parametrize('speed, moving', [(0.0, False), (0.1, False), (0.6, True)])
def test_only_obstacles_faster_than_threshold_are_moving(speed, moving):
    tracker = MotionTracker()
    now = walk(tracker, (3.0, 0.0), (0.0, speed), 30)
    assert bool(tracker.dynamic(now)) is moving
    assert tracker.tracks[0].speed == pytest.approx(speed, abs=0.08)


def test_paused_obstacle_stays_dynamic_for_memory_time():
    tracker = MotionTracker()
    now = walk(tracker, (3.0, 0.0), (0.0, 0.6), 20)
    stop = walk(tracker, (3.0, 1.14), (0.0, 0.0), 15, t0=now + 0.1)
    assert tracker.dynamic(stop)  # paused 1.4 s ago: still treated as dynamic
    later = walk(tracker, (3.0, 1.14), (0.0, 0.0), 40, t0=stop + 0.1)
    assert not tracker.dynamic(later)


def test_gate_stops_waits_and_resumes_with_hysteresis():
    gate = YieldGate()
    assert gate.update(0.0, [(2.0, 0.0)])            # moving but far: go
    assert not gate.update(0.1, [(0.9, 0.2)])        # within 1 m ahead: stop
    assert gate.state == YIELDING
    assert not gate.update(1.0, [(1.3, 0.0)])        # 1.3 m < resume 1.5: wait
    assert not gate.update(1.5, [(1.7, 0.0)])        # clear, hold not elapsed
    assert gate.update(2.6, [(1.8, 0.0)])            # clear for > 1 s: go
    assert gate.state == CLEAR


def test_gate_ignores_obstacles_behind_unless_reversing():
    gate = YieldGate()
    assert gate.update(0.0, [(-0.6, 0.0)])
    assert not gate.update(0.1, [(-0.6, 0.0)], reversing=True)


def test_gate_times_out_then_rearms():
    gate = YieldGate(YieldConfig(max_wait=5.0))
    assert not gate.update(0.0, [(0.8, 0.0)])
    assert not gate.update(4.9, [(0.8, 0.0)])
    assert gate.update(5.1, [(0.8, 0.0)])            # waited too long: drive on
    assert gate.state == TIMED_OUT
    assert gate.update(6.0, [(0.8, 0.0)])            # stays released while there
    gate.update(7.0, [])
    gate.update(8.1, [])
    assert gate.state == CLEAR
    assert not gate.update(8.2, [(0.9, 0.0)])        # next encounter stops again


def test_config_rejects_resume_inside_stop():
    with pytest.raises(ValueError):
        YieldConfig(stop_distance=1.5, resume_distance=1.0)


def test_scan_pose_is_interpolated_from_odometry():
    from warehouse_mission_manager.dynamic_yield_node import interpolate
    history = [(0.0, 0.0, 0.0, 3.1), (0.1, 0.1, 0.0, -3.1)]
    x, y, yaw = interpolate(history, 0.05)
    assert x == pytest.approx(0.05)
    assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) == pytest.approx(math.pi, abs=1e-3)
    assert interpolate(history, 0.15) == history[-1][1:]   # <= 0.1 s ahead: hold
    assert interpolate(history, 0.3) is None               # too stale
    assert interpolate(history, -0.1) is None
