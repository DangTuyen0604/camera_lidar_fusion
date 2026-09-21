"""Short headless warehouse smoke test for core runtime signals."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import FusedDetectionArray
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
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    bringup = Path(get_package_share_directory('navigation_bringup'))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup / 'launch' / 'warehouse_full_demo.launch.py')),
        launch_arguments={
            'gui': 'false',
            'mission_autostart': 'false',
        }.items(),
    )
    return launch.LaunchDescription([
        stack,
        launch_testing.actions.ReadyToTest(),
    ])


class TestWarehouseSmoke(unittest.TestCase):
    """Verify that the headless stack reaches a usable idle state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('warehouse_smoke_test')
        self.received = set()
        self.subscriptions = []
        for message_type, topic in (
                (Clock, '/clock'),
                (Odometry, '/odom'),
                (JointState, '/joint_states'),
                (LaserScan, '/scan'),
                (Imu, '/imu'),
                (Image, '/camera/image_raw'),
                (CameraInfo, '/camera/camera_info'),
                (TFMessage, '/tf'),
                (FusedDetectionArray, '/fusion/detections_3d')):
            subscription = self.node.create_subscription(
                message_type,
                topic,
                lambda unused, topic=topic: self.received.add(topic),
                qos_profile_sensor_data,
            )
            self.subscriptions.append(subscription)
        self.navigation = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.mission = self.node.create_client(Trigger, '/mission/start')

    def tearDown(self):
        self.navigation.destroy()
        self.node.destroy_node()

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_core_runtime_becomes_ready(self):
        expected_topics = {
            '/clock',
            '/odom',
            '/joint_states',
            '/scan',
            '/imu',
            '/camera/image_raw',
            '/camera/camera_info',
            '/tf',
            '/fusion/detections_3d',
        }
        self.assertTrue(
            self.spin_until(lambda: expected_topics <= self.received, 90.0),
            f'missing runtime messages: {sorted(expected_topics - self.received)}',
        )
        self.assertTrue(self.navigation.wait_for_server(timeout_sec=45.0))
        self.assertTrue(self.mission.wait_for_service(timeout_sec=20.0))
