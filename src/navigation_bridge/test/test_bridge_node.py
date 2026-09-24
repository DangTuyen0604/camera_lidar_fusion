"""Black-box tests for timestamps, TF failure, timeout clearing and recovery."""

import math
import struct
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from fusion_interfaces.msg import FusedDetection, FusedDetectionArray
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def generate_test_description():
    bridge = launch_ros.actions.Node(
        package='navigation_bridge', executable='detection_obstacle_bridge_node',
        parameters=[{'stale_timeout': 0.4, 'future_tolerance': 0.1,
                     'obstacle_timeout': 0.3, 'tf_timeout': 0.02}])
    transform = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '1', '--y', '0', '--z', '0', '--frame-id', 'base_link',
                   '--child-frame-id', 'camera_optical_frame'])
    return launch.LaunchDescription([
        bridge, transform, launch_testing.actions.ReadyToTest()]), {'bridge': bridge}


class TestBridgeNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_detection_obstacle_bridge')
        cls.clouds = []
        cls.clearings = []
        cls.diagnostics = []
        cls.publisher = cls.node.create_publisher(
            FusedDetectionArray, '/fusion/detections_3d', 10)
        cls.node.create_subscription(
            PointCloud2, '/navigation/detection_obstacles', cls.clouds.append,
            qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, '/navigation/detection_obstacles/clearing', cls.clearings.append,
            qos_profile_sensor_data)
        cls.node.create_subscription(DiagnosticArray, '/diagnostics', cls.diagnostics.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_for(self, message, duration=0.25):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def diagnostic_values(message):
        for status in message.status:
            if status.name == 'navigation_bridge/detection_obstacle_bridge':
                return {item.key: int(item.value) for item in status.values}
        return {}

    @staticmethod
    def xyz_points(cloud):
        endian = '>' if cloud.is_bigendian else '<'
        offsets = {field.name: field.offset for field in cloud.fields}
        points = []
        for index in range(cloud.width * cloud.height):
            base = index * cloud.point_step
            points.append(tuple(struct.unpack_from(
                f'{endian}f', cloud.data, base + offsets[name])[0]
                for name in ('x', 'y', 'z')))
        return points

    def message(self, frame='camera_optical_frame', offset=0.0, valid=True):
        message = FusedDetectionArray()
        message.header.frame_id = frame
        stamp = self.node.get_clock().now() + Duration(seconds=offset)
        message.header.stamp = stamp.to_msg()
        detection = FusedDetection()
        detection.valid = valid
        detection.detection.class_name = 'pallet'
        detection.detection.confidence = 0.9
        detection.position.x = 2.0
        message.detections = [detection]
        return message

    def test_00_publishes_empty_heartbeat_before_first_detection(self):
        self.assertTrue(self.spin_until(lambda: bool(self.clouds)))
        cloud = self.clouds[-1]
        self.assertEqual(cloud.header.frame_id, 'base_link')
        self.assertEqual(cloud.width, 0)

    def test_01_rejections_do_not_kill_node(self):
        nonempty_before = sum(cloud.width > 0 for cloud in self.clouds)
        self.publish_for(self.message(valid=False), 0.15)
        non_finite = self.message()
        non_finite.detections[0].position.y = math.inf
        self.publish_for(non_finite, 0.15)
        nan_detection = self.message()
        nan_detection.detections[0].position.z = math.nan
        self.publish_for(nan_detection, 0.15)
        absurd = self.message()
        absurd.detections[0].position.x = 1000.0
        self.publish_for(absurd, 0.15)
        self.publish_for(self.message(frame=''), 0.15)

        zero_stamp = self.message()
        zero_stamp.header.stamp.sec = 0
        zero_stamp.header.stamp.nanosec = 0
        self.publish_for(zero_stamp, 0.15)
        self.publish_for(self.message(offset=-5.0), 0.15)
        self.publish_for(self.message(offset=5.0), 0.15)
        self.publish_for(self.message(frame='missing_frame'), 0.15)

        self.assertEqual(
            sum(cloud.width > 0 for cloud in self.clouds), nonempty_before)
        diagnostics_after_inputs = len(self.diagnostics)
        expected_keys = (
            'rejected_invalid', 'rejected_range', 'rejected_non_finite',
            'rejected_empty_frame', 'rejected_zero_stamp', 'rejected_stale',
            'rejected_future', 'rejected_missing_tf')
        self.assertTrue(self.spin_until(
            lambda: any(
                all(key in self.diagnostic_values(message)
                    for key in expected_keys)
                for message in self.diagnostics[diagnostics_after_inputs:]),
            2.0))
        values = next(
            values for values in (
                self.diagnostic_values(message)
                for message in reversed(
                    self.diagnostics[diagnostics_after_inputs:]))
            if all(key in values for key in expected_keys))
        for key in expected_keys:
            self.assertGreater(values[key], 0, key)

    def test_02_valid_transform_and_timeout_clearing(self):
        before = len(self.clouds)
        for _ in range(3):
            self.publisher.publish(self.message())
            rclpy.spin_once(self.node, timeout_sec=0.10)
        self.assertTrue(self.spin_until(
            lambda: any(cloud.width > 0 for cloud in self.clouds[before:])))
        cloud = next(
            cloud for cloud in reversed(self.clouds[before:])
            if cloud.width > 0)
        self.assertEqual(cloud.header.frame_id, 'base_link')
        self.assertGreater(cloud.width, 1)  # expanded pallet footprint
        points = self.xyz_points(cloud)
        # Static TF adds +1 m in x. A pallet centered at camera x=2 m spans
        # x=[2.4, 3.6] and y=[-0.4, 0.4] in base_link.
        self.assertAlmostEqual(min(point[0] for point in points), 2.4, places=4)
        self.assertAlmostEqual(max(point[0] for point in points), 3.6, places=4)
        self.assertAlmostEqual(min(point[1] for point in points), -0.4, places=4)
        self.assertAlmostEqual(max(point[1] for point in points), 0.4, places=4)

        # Repeated invalid observations must not refresh the valid obstacle's
        # lifetime. The bridge must still emit clearing rays and an empty cloud.
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            self.publisher.publish(self.message(valid=False))
            rclpy.spin_once(self.node, timeout_sec=0.05)
        expired_clouds = len(self.clouds)
        self.assertTrue(self.spin_until(
            lambda: bool(self.clearings) and
            sum(item.width == 0 for item in self.clouds[expired_clouds:]) >= 2,
            2.0))
        self.assertEqual(len(self.clearings), 1)


@launch_testing.post_shutdown_test()
class TestBridgeExit(unittest.TestCase):

    def test_process_survived_bad_input(self, proc_info, bridge):
        launch_testing.asserts.assertExitCodes(proc_info, process=bridge)
