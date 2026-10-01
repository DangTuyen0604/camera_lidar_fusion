"""Detect moving obstacles in a planar LiDAR scan (no ROS, unit tested).

Pipeline per scan:
1. Drop returns that lie on the static map (walls, racks): only unmapped
   objects can be dynamic.
2. Split the remaining returns, in scan order, into clusters wherever two
   consecutive points are farther apart than cluster_gap.
3. Associate cluster centroids with existing tracks (nearest neighbour
   within association_gate) and update an alpha-beta (constant velocity)
   filter per track.  Tracks live in a fixed frame (odom) so the robot's
   own motion does not look like obstacle motion.
4. A track is MOVING once its filtered speed stayed above moving_speed for
   moving_frames consecutive scans; it stays "recently moving" for
   memory_time seconds after it last moved, so a person who pauses next to
   the robot is still treated as dynamic.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class TrackerConfig:
    cluster_gap: float = 0.25          # m between consecutive returns
    min_points: int = 3
    max_extent: float = 1.5            # m; bigger clusters are not objects
    association_gate: float = 0.6      # m
    alpha: float = 0.5
    beta: float = 0.2
    max_missed: int = 5                # scans a track survives unseen
    moving_speed: float = 0.35         # m/s
    moving_frames: int = 3
    memory_time: float = 3.0           # s


@dataclass
class Track:
    id: int
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    stamp: float = 0.0
    missed: int = 0
    fast_frames: int = 0
    last_moving: float = None

    @property
    def speed(self):
        return float(np.hypot(*self.velocity))

    def recently_moving(self, now, memory_time):
        return self.last_moving is not None and now - self.last_moving <= memory_time


def clusters(points, config):
    """Centroids of object-sized clusters of (N, 2) points given in scan order."""
    if len(points) == 0:
        return []
    gaps = np.hypot(*np.diff(points, axis=0).T) > config.cluster_gap
    groups = np.split(points, np.nonzero(gaps)[0] + 1)
    if len(groups) > 1 and math.hypot(*(groups[0][0] - groups[-1][-1])) <= config.cluster_gap:
        groups = [np.vstack([groups[-1], groups[0]])] + groups[1:-1]  # 360 deg wrap
    centroids = []
    for group in groups:
        extent = math.hypot(*(group.max(axis=0) - group.min(axis=0)))
        if len(group) >= config.min_points and extent <= config.max_extent:
            centroids.append(group.mean(axis=0))
    return centroids


class MotionTracker:

    def __init__(self, config=None):
        self.config = config or TrackerConfig()
        self.tracks = []
        self._next_id = 0

    def update(self, now, centroids):
        """Feed one scan's cluster centroids (fixed frame); return the tracks."""
        cfg = self.config
        unmatched = list(range(len(centroids)))
        for track in sorted(self.tracks, key=lambda t: t.missed):
            dt = now - track.stamp
            predicted = track.position + track.velocity * dt
            best, best_distance = None, cfg.association_gate
            for index in unmatched:
                distance = math.hypot(*(centroids[index] - predicted))
                if distance < best_distance:
                    best, best_distance = index, distance
            if best is None:
                track.missed += 1
                continue
            unmatched.remove(best)
            residual = centroids[best] - predicted
            track.position = predicted + cfg.alpha * residual
            if dt > 0.0:
                track.velocity = track.velocity + cfg.beta * residual / dt
            track.stamp, track.missed = now, 0
            track.fast_frames = track.fast_frames + 1 if track.speed > cfg.moving_speed else 0
            if track.fast_frames >= cfg.moving_frames:
                track.last_moving = now
        self.tracks = [t for t in self.tracks if t.missed <= cfg.max_missed]
        for index in unmatched:
            self.tracks.append(Track(self._next_id, np.array(centroids[index], float),
                                     stamp=now))
            self._next_id += 1
        return self.tracks

    def dynamic(self, now):
        """Tracks that move now or moved within memory_time."""
        return [t for t in self.tracks if t.recently_moving(now, self.config.memory_time)]
