"""Black-box smoke and interface test for the one-command final demo."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import (
    Detection2DArray, FusedDetectionArray, PipelineMetrics, SyncStatus)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    """Start the exact final launch with graphical clients disabled for CI."""
    share = Path(get_package_share_directory('fusion_bringup'))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'final_demo.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items())
    return launch.LaunchDescription([
        stack,
        launch_testing.actions.ReadyToTest(),
    ])


class TestCanonicalLaunchSmoke(unittest.TestCase):
    """Observe live messages, graph interfaces, TF and active navigation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('canonical_launch_smoke_test')
        self.received = set()
        self.subscriptions = []
        for message_type, topic, qos in (
                (Clock, '/clock', qos_profile_sensor_data),
                (Odometry, '/odom', qos_profile_sensor_data),
                (LaserScan, '/scan_filtered', qos_profile_sensor_data),
                (Image, '/camera/image_raw', qos_profile_sensor_data),
                (CameraInfo, '/camera/camera_info', qos_profile_sensor_data),
                (PointCloud2, '/lidar/points', qos_profile_sensor_data),
                (Image, '/fusion/synced/image', qos_profile_sensor_data),
                (Detection2DArray, '/detections_2d', 10),
                (FusedDetectionArray, '/fusion/detections_3d', 10),
                (SyncStatus, '/fusion/sync_status', 10),
                (PipelineMetrics, '/fusion/metrics', 10),
                (TFMessage, '/tf', qos_profile_sensor_data)):
            self.subscriptions.append(self.node.create_subscription(
                message_type, topic,
                lambda unused, name=topic: self.received.add(name), qos))
        self.navigation = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose')

    def tearDown(self):
        self.navigation.destroy()
        self.node.destroy_node()

    def wait(self, predicate, timeout):
        """Spin until a runtime condition is observable."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_final_launch_topics_nodes_and_interfaces(self):
        expected_topics = {
            '/clock', '/odom', '/scan_filtered', '/camera/image_raw',
            '/camera/camera_info', '/lidar/points', '/fusion/synced/image',
            '/detections_2d', '/fusion/detections_3d',
            '/fusion/sync_status', '/fusion/metrics', '/tf',
        }
        self.assertTrue(
            self.wait(lambda: expected_topics <= self.received, 90.0),
            f'missing live topics: {sorted(expected_topics - self.received)}')
        self.assertTrue(self.navigation.wait_for_server(timeout_sec=45.0))

        topic_types = dict(self.node.get_topic_names_and_types())
        expected_interfaces = {
            '/detections_2d': 'fusion_interfaces/msg/Detection2DArray',
            '/fusion/detections_3d':
                'fusion_interfaces/msg/FusedDetectionArray',
            '/navigation/detection_obstacles': 'sensor_msgs/msg/PointCloud2',
            '/cmd_vel_nav': 'geometry_msgs/msg/Twist',
            '/cmd_vel_smoothed': 'geometry_msgs/msg/Twist',
            '/cmd_vel': 'geometry_msgs/msg/Twist',
        }
        for topic, interface in expected_interfaces.items():
            self.assertIn(topic, topic_types)
            self.assertIn(interface, topic_types[topic])

        node_names = set(self.node.get_node_names())
        expected_nodes = {
            'robot_state_publisher', 'sensor_sync_node',
            'simulation_color_detector_node', 'object_fusion_node',
            'detection_obstacle_bridge', 'metrics_node', 'controller_server',
            'velocity_smoother', 'collision_monitor',
        }
        self.assertTrue(
            expected_nodes <= node_names,
            f'missing nodes: {sorted(expected_nodes - node_names)}')
