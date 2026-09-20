"""Black-box tests for timestamps, TF failure, timeout clearing and recovery."""

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

    def test_01_rejections_do_not_kill_node(self):
        empty_frame = self.message(frame='')
        zero_stamp = self.message()
        zero_stamp.header.stamp.sec = 0
        zero_stamp.header.stamp.nanosec = 0
        for message in (empty_frame, zero_stamp, self.message(offset=-1.0),
                        self.message(offset=1.0), self.message(frame='missing_frame')):
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.10)
        self.assertTrue(self.spin_until(lambda: bool(self.diagnostics), 2.0))

    def test_02_valid_transform_and_timeout_clearing(self):
        before = len(self.clouds)
        for _ in range(3):
            self.publisher.publish(self.message())
            rclpy.spin_once(self.node, timeout_sec=0.10)
        self.assertTrue(self.spin_until(lambda: len(self.clouds) > before))
        cloud = self.clouds[-1]
        self.assertEqual(cloud.header.frame_id, 'base_link')
        self.assertGreater(cloud.width, 1)  # expanded pallet footprint
        self.assertTrue(self.spin_until(
            lambda: bool(self.clearings) and any(item.width == 0 for item in self.clouds),
            2.0))


@launch_testing.post_shutdown_test()
class TestBridgeExit(unittest.TestCase):

    def test_process_survived_bad_input(self, proc_info, bridge):
        launch_testing.asserts.assertExitCodes(proc_info, process=bridge)
