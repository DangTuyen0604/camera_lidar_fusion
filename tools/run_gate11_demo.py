#!/usr/bin/env python3
"""Run one canonical localization-perception-navigation demo."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import (
    Detection2DArray, FusedDetectionArray, PipelineMetrics, SyncStatus)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image, PointCloud2
from simulation_interfaces.srv import DeleteEntity, SpawnEntity


class Gate11Probe(Node):
    """Drive one Nav2 goal while recording every Gate 11 pipeline boundary."""

    def __init__(self, seed):
        super().__init__('gate11_runtime_probe')
        self.seed = seed
        self.detections = []
        self.fused = []
        self.obstacles = []
        self.nav_commands = []
        self.smoothed_commands = []
        self.output_commands = []
        self.monitor_states = []
        self.positions = []
        self.camera_times = []
        self.pointcloud_times = []
        self.detection_times = []
        self.fusion_times = []
        self.sync_offsets_ms = []
        self.pipeline_metrics = []
        self.plans = []
        self.localized = False
        self.map_occupied_cells = 0
        self.worker_spawned_at = None
        self.first_detection_at = None
        self.first_fusion_at = None
        self.first_obstacle_at = None
        self.first_reaction_at = None
        self.worker_removed_at = None
        self.first_empty_after_removal_at = None
        self.goal_started_at = None
        self.worker_present = False
        self._gate_subscriptions = []
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = (
            (Detection2DArray, '/detections_2d', self._detection, 10),
            (FusedDetectionArray, '/fusion/detections_3d', self._fusion, 10),
            (PointCloud2, '/navigation/detection_obstacles', self._obstacle,
             qos_profile_sensor_data),
            (Twist, '/cmd_vel_nav', self.nav_commands.append, 20),
            (Twist, '/cmd_vel_smoothed', self.smoothed_commands.append, 20),
            (Twist, '/cmd_vel', self.output_commands.append, 20),
            (CollisionMonitorState, '/collision_monitor_state', self._monitor, 20),
            (Image, '/camera/image_raw', self._camera, qos_profile_sensor_data),
            (PointCloud2, '/lidar/points', self._pointcloud,
             qos_profile_sensor_data),
            (SyncStatus, '/fusion/sync_status', self._sync, 20),
            (PipelineMetrics, '/fusion/metrics', self.pipeline_metrics.append, 20),
            (NavPath, '/plan', self.plans.append, 10),
            (PoseWithCovarianceStamped, '/amcl_pose', self._localization, 10),
            (OccupancyGrid, '/map', self._map, map_qos),
        )
        for message_type, topic, callback, qos in subscriptions:
            self._gate_subscriptions.append(self.create_subscription(
                message_type, topic, callback, qos))
        self._gate_subscriptions.append(self.create_subscription(
            Odometry, '/odom', self._odom, 20))
        self.spawn_client = self.create_client(
            SpawnEntity, '/gzserver/spawn_entity')
        self.delete_client = self.create_client(
            DeleteEntity, '/gzserver/delete_entity')
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.minimum_clearance = math.inf

    def _camera(self, unused_message):
        self.camera_times.append(time.monotonic())

    def _pointcloud(self, unused_message):
        self.pointcloud_times.append(time.monotonic())

    def _detection(self, message):
        now = time.monotonic()
        self.detections.append(message)
        self.detection_times.append(now)
        if message.detections and self.first_detection_at is None:
            self.first_detection_at = now

    def _fusion(self, message):
        now = time.monotonic()
        self.fused.append(message)
        self.fusion_times.append(now)
        if (any(item.valid for item in message.detections) and
                self.first_fusion_at is None):
            self.first_fusion_at = now

    def _obstacle(self, message):
        now = time.monotonic()
        self.obstacles.append(message)
        if message.width and self.first_obstacle_at is None:
            self.first_obstacle_at = now
        if (not message.width and self.worker_removed_at is not None and
                self.first_empty_after_removal_at is None):
            self.first_empty_after_removal_at = now

    def _monitor(self, message):
        self.monitor_states.append(message)
        if (message.action_type in (
                CollisionMonitorState.STOP, CollisionMonitorState.SLOWDOWN) and
                self.first_reaction_at is None):
            self.first_reaction_at = time.monotonic()

    def _sync(self, message):
        self.sync_offsets_ms.append(abs(float(message.camera_lidar_offset_ms)))

    def _localization(self, unused_message):
        self.localized = True

    def _map(self, message):
        self.map_occupied_cells = sum(value >= 65 for value in message.data)

    def _odom(self, message):
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        self.positions.append((x, y))
        if self.worker_present:
            self.minimum_clearance = min(
                self.minimum_clearance, math.hypot(1.2 - x, y))

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        if not self.wait(future.done, timeout):
            raise RuntimeError(f'service timeout: {client.srv_name}')
        return future.result()

    def spawn_worker(self):
        request = SpawnEntity.Request()
        request.name = 'gate11_worker'
        request.uri = (Path(get_package_share_directory('warehouse_simulation')) /
                       'models' / 'worker' / 'model.sdf').as_uri()
        request.initial_pose.header.frame_id = 'world'
        request.initial_pose.pose.position.x = 1.2
        request.initial_pose.pose.position.y = -4.0
        request.initial_pose.pose.orientation.w = 1.0
        # Start the latency clock before the service call: Gazebo may publish
        # the new entity to sensors before its response reaches this node.
        self.worker_spawned_at = time.monotonic()
        response = self.call(self.spawn_client, request)
        if response.result.result != response.result.RESULT_OK:
            raise RuntimeError(response.result.error_message)
        self.worker_present = True

    def delete_worker(self):
        request = DeleteEntity.Request()
        request.entity = 'gate11_worker'
        response = self.call(self.delete_client, request)
        if response.result.result != response.result.RESULT_OK:
            raise RuntimeError(response.result.error_message)
        self.worker_present = False
        self.worker_removed_at = time.monotonic()

    def send_goal(self):
        if not self.navigation.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('navigate_to_pose action unavailable')
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = 2.5
            goal.pose.pose.orientation.w = 1.0
            future = self.navigation.send_goal_async(goal)
            if not self.wait(future.done, 10.0):
                raise RuntimeError('goal response timeout')
            handle = future.result()
            if handle.accepted:
                self.goal_started_at = time.monotonic()
                return handle.get_result_async()
            self.wait(lambda: False, 0.5)
        raise RuntimeError('navigate_to_pose stayed inactive for 30 seconds')


def moving(command):
    return abs(command.linear.x) > 0.05 or abs(command.angular.z) > 0.05


def message_rate(times):
    """Calculate arrival rate without inventing samples."""
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--run', type=int, default=1)
    parser.add_argument('--seed', type=int, default=12012)
    args = parser.parse_args()
    rclpy.init()
    probe = Gate11Probe(args.seed)
    spawned = False
    try:
        if not probe.wait(
                lambda: (probe.positions and probe.localized and
                         probe.map_occupied_cells > 0), 45.0):
            raise RuntimeError('localization, odometry, or static map unavailable')

        result_future = probe.send_goal()
        if not probe.wait(
                lambda: any(moving(item)
                            for item in probe.smoothed_commands), 15.0):
            raise RuntimeError('controller command chain never requested motion')
        if not probe.wait(lambda: any(plan.poses for plan in probe.plans), 5.0):
            raise RuntimeError('Nav2 did not produce a path through the static map')

        # Place the physical target only after motion starts. This makes the
        # measured response attributable to live perception, not startup.
        probe.first_detection_at = None
        probe.first_fusion_at = None
        probe.first_obstacle_at = None
        probe.first_reaction_at = None
        probe.spawn_worker()
        spawned = True
        if not probe.wait(
                lambda: probe.first_detection_at is not None,
                20.0):
            raise RuntimeError('camera detector did not see worker')
        if not probe.wait(lambda: probe.first_fusion_at is not None, 20.0):
            raise RuntimeError('no valid fused XYZ')
        if not probe.wait(lambda: probe.first_obstacle_at is not None, 10.0):
            raise RuntimeError('bridge did not publish obstacle')
        reacted = probe.wait(
            lambda: probe.first_reaction_at is not None, 15.0)
        if not reacted:
            raise RuntimeError('robot did not react to perception obstacle')

        valid = next(
            detection for message in reversed(probe.fused)
            for detection in message.detections if detection.valid)
        probe.delete_worker()
        spawned = False
        obstacle_count = len(probe.obstacles)
        if not probe.wait(lambda: sum(
                message.width == 0
                for message in probe.obstacles[obstacle_count:]) >= 2, 5.0):
            raise RuntimeError('bridge did not publish empty heartbeat after expiry')
        if not probe.wait(result_future.done, 60.0):
            raise RuntimeError('goal did not finish after obstacle removal')
        result = result_future.result()
        if result.status != 4:
            raise RuntimeError(f'goal failed with status {result.status}')
        goal_finished_at = time.monotonic()
        if not probe.sync_offsets_ms or not probe.pipeline_metrics:
            raise RuntimeError('sync or pipeline metrics were not observed')

        trace = {
            'scenario': 'canonical_perception_navigation',
            'run': args.run,
            'seed': args.seed,
            'result': 'PASS',
            'camera_detection': {
                'class': valid.detection.class_name,
                'confidence': round(float(valid.detection.confidence), 3),
            },
            'fused_xyz_camera_optical': [
                round(valid.position.x, 3),
                round(valid.position.y, 3),
                round(valid.position.z, 3),
            ],
            'lidar_point_count': valid.lidar_point_count,
            'bridge_nonempty_clouds': sum(
                message.width > 0 for message in probe.obstacles),
            'collision_actions': [
                state.action_type for state in probe.monitor_states],
            'max_controller_speed': round(max(
                abs(item.linear.x) for item in probe.nav_commands), 3),
            'max_smoothed_speed': round(max(
                abs(item.linear.x) for item in probe.smoothed_commands), 3),
            'minimum_output_speed': round(min(
                abs(item.linear.x) for item in probe.output_commands), 3),
            'minimum_robot_obstacle_clearance_m': round(
                probe.minimum_clearance, 3),
            'collision_count': 0 if probe.minimum_clearance > 0.62 else 1,
            'goal_status': 'SUCCEEDED',
            'goal_duration_s': round(
                goal_finished_at - probe.goal_started_at, 3),
            'obstacle_reaction_latency_ms': round(
                (probe.first_reaction_at - probe.worker_spawned_at) * 1000.0,
                3),
            'camera_rate_hz': round(message_rate(probe.camera_times), 3),
            'pointcloud_rate_hz': round(
                message_rate(probe.pointcloud_times), 3),
            'detection_rate_hz': round(
                message_rate(probe.detection_times), 3),
            'fusion_rate_hz': round(message_rate(probe.fusion_times), 3),
            'sync_delay_mean_ms': round(
                statistics.fmean(probe.sync_offsets_ms), 3),
            'pipeline_end_to_end_mean_ms': round(statistics.fmean(
                item.end_to_end_ms for item in probe.pipeline_metrics), 3),
            'obstacle_expiry_s': round(
                probe.first_empty_after_removal_at - probe.worker_removed_at,
                3),
            'evidence': {
                'localized': probe.localized,
                'static_map_occupied_cells': probe.map_occupied_cells,
                'nav_paths_received': len(probe.plans),
                'camera_frames': len(probe.camera_times),
                'pointcloud_frames': len(probe.pointcloud_times),
                'detections_received': len(probe.detections),
                'fused_messages_received': len(probe.fused),
                'empty_bridge_heartbeats_after_removal': sum(
                    message.width == 0
                    for message in probe.obstacles[obstacle_count:]),
            },
        }
        rendered = json.dumps(trace, indent=2, sort_keys=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + '\n', encoding='utf-8')
        print(rendered)
    finally:
        if spawned:
            try:
                probe.delete_worker()
            except RuntimeError:
                pass
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
