#!/usr/bin/env python3
"""Collect benchmark metrics from live ROS topics; never synthesize samples."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import (
    CalibrationStatus, FusedDetectionArray, PipelineMetrics)
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt64
import yaml


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = min(len(values) - 1, math.ceil(fraction * len(values)) - 1)
    return values[index]


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('benchmark_metrics_collector')
        self.latencies = []
        self.projection_errors = []
        self.calibration_scores = []
        self.fused = self.total = 0
        self.path_signatures = []
        self.stop_events = 0
        self.collisions = None
        self.mission_started = self.mission_finished = None
        self.mission_success = False
        self.person_seen = self.person_stopped = False
        self.docking_error = None
        stations_file = (Path(get_package_share_directory('warehouse_mission_manager')) /
                         'config' / 'stations.yaml')
        self.stations = yaml.safe_load(stations_file.read_text())['stations']
        self.truth = []
        self.xyz_errors = []
        self.last_odom = None
        self.obstacle_last_nonempty = self.obstacle_first_empty = None
        self.create_subscription(PipelineMetrics, '/fusion/metrics', self._metrics, 20)
        self.create_subscription(CalibrationStatus, '/fusion/calibration_status',
                                 self._calibration, 20)
        self.create_subscription(FusedDetectionArray, '/fusion/detections_3d',
                                 self._fusion, 20)
        self.create_subscription(FusedDetectionArray,
                                 '/benchmark/ground_truth/detections_3d',
                                 self._truth, 20)
        self.create_subscription(NavPath, '/plan', self._path, 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self._stop, 10)
        self.create_subscription(UInt64, '/benchmark/collision_count',
                                 self._collision, 10)
        self.create_subscription(String, '/mission/state', self._mission, 20)
        self.create_subscription(String, '/benchmark/events', self._event, 20)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(PointCloud2, '/navigation/detection_obstacles',
                                 self._obstacle, 20)
        self.create_subscription(Twist, '/cmd_vel', self._velocity, 20)

    def _metrics(self, msg):
        self.latencies.append(float(msg.end_to_end_ms))

    def _calibration(self, msg):
        self.projection_errors.append(float(msg.projection_error_px))
        self.calibration_scores.append(float(msg.alignment_score))

    def _fusion(self, msg):
        self.total += len(msg.detections)
        self.fused += sum(item.valid for item in msg.detections)
        for detection in msg.detections:
            candidates = [truth for truth in self.truth
                          if truth.detection.class_id == detection.detection.class_id]
            if detection.valid and candidates:
                error = min(math.sqrt(
                    (detection.position.x - truth.position.x) ** 2 +
                    (detection.position.y - truth.position.y) ** 2 +
                    (detection.position.z - truth.position.z) ** 2)
                    for truth in candidates)
                self.xyz_errors.append(error)

    def _truth(self, msg):
        self.truth = list(msg.detections)

    def _path(self, msg):
        signature = tuple((round(p.pose.position.x, 2), round(p.pose.position.y, 2))
                          for p in msg.poses)
        if signature and (not self.path_signatures or
                          signature != self.path_signatures[-1]):
            self.path_signatures.append(signature)

    def _stop(self, msg):
        if msg.action_type == CollisionMonitorState.STOP:
            self.stop_events += 1
            if self.person_seen:
                self.person_stopped = True

    def _collision(self, msg):
        self.collisions = int(msg.data)

    def _mission(self, msg):
        now = time.monotonic()
        if msg.data != 'IDLE' and self.mission_started is None:
            self.mission_started = now
        if msg.data == 'COMPLETED':
            self.mission_finished = now
            self.mission_success = True
        if msg.data == 'DOCKED' and self.last_odom is not None:
            point = self.last_odom.pose.pose.position
            self.docking_error = min(
                math.hypot(point.x - station['x'], point.y - station['y'])
                for station in self.stations.values())

    def _event(self, msg):
        if 'spawn:crossing_worker' in msg.data:
            self.person_seen = True

    def _odom(self, msg):
        self.last_odom = msg

    def _obstacle(self, msg):
        now = time.monotonic()
        if msg.width:
            self.obstacle_last_nonempty = now
            self.obstacle_first_empty = None
        elif self.obstacle_last_nonempty is not None and self.obstacle_first_empty is None:
            self.obstacle_first_empty = now

    def _velocity(self, msg):
        if self.person_seen and math.hypot(msg.linear.x, msg.linear.y) < 0.02:
            self.person_stopped = True

    def ready(self):
        return bool(self.latencies and self.projection_errors and self.total and
                    self.mission_started is not None and self.collisions is not None)

    def result(self):
        required = {
            'latency_mean_ms': statistics.fmean(self.latencies) if self.latencies else None,
            'latency_p95_ms': percentile(self.latencies, 0.95),
            'latency_p99_ms': percentile(self.latencies, 0.99),
            'projection_error_px': (statistics.fmean(self.projection_errors)
                                    if self.projection_errors else None),
            'xyz_error_m': (statistics.fmean(self.xyz_errors)
                            if self.xyz_errors else None),
            'fusion_success_rate': self.fused / self.total if self.total else None,
            'calibration_quality': (statistics.fmean(self.calibration_scores)
                                    if self.calibration_scores else None),
            'replan_count': max(0, len(self.path_signatures) - 1),
            'stop_events': self.stop_events,
            'collision_count': self.collisions,
            'mission_duration_s': (self.mission_finished - self.mission_started
                                   if self.mission_finished else None),
            'mission_success': int(self.mission_success),
            'person_stop_success': int(self.person_seen and self.person_stopped),
            'docking_error_m': self.docking_error,
            'ghost_obstacle_lifetime_s': (
                self.obstacle_first_empty - self.obstacle_last_nonempty
                if self.obstacle_first_empty else None),
        }
        missing = [key for key, value in required.items() if value is None]
        if missing:
            raise RuntimeError(f'missing live measurements: {missing}')
        return required


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=90.0)
    parser.add_argument('--startup-timeout', type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = MetricsCollector()
    start = time.monotonic()
    collecting = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if collecting is None and node.ready():
                collecting = now
            if collecting is None and now - start > args.startup_timeout:
                raise RuntimeError('required benchmark topics did not become ready')
            if collecting is not None and now - collecting >= args.duration:
                break
        result = node.result()
        Path(args.output).write_text(json.dumps(result, indent=2), encoding='utf-8')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
