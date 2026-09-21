"""Live 18-step Day-6 warehouse acceptance test."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import FusedDetectionArray
from geometry_msgs.msg import Twist
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    bringup = Path(get_package_share_directory('navigation_bringup'))
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup / 'launch' / 'warehouse_full_demo.launch.py')),
        launch_arguments={'gui': 'false', 'mission_autostart': 'false'}.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestWarehouseEndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('warehouse_end_to_end_test')
        self.paths = []
        self.fused = []
        self.obstacles = []
        self.states = []
        self.cargo = []
        self.events = []
        self.velocities = []
        self.collisions = []
        for message, topic, target in (
                (NavPath, '/plan', self.paths),
                (FusedDetectionArray, '/fusion/detections_3d', self.fused),
                (String, '/mission/state', self.states),
                (String, '/mission/cargo', self.cargo),
                (String, '/benchmark/events', self.events),
                (Twist, '/cmd_vel', self.velocities),
                (UInt64, '/benchmark/collision_count', self.collisions)):
            self.node.create_subscription(message, topic, target.append, 20)
        self.node.create_subscription(
            PointCloud2,
            '/navigation/detection_obstacles',
            self.obstacles.append,
            qos_profile_sensor_data,
        )
        self.start_client = self.node.create_client(Trigger, '/mission/start')
        self.navigation = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def tearDown(self):
        self.node.destroy_node()

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    @staticmethod
    def signature(path):
        return tuple((round(item.pose.position.x, 2),
                      round(item.pose.position.y, 2)) for item in path.poses)

    def test_complete_warehouse_mission(self):
        # 1-3: launch headless, wait for active Nav2, send the mission.
        self.assertTrue(self.navigation.wait_for_server(timeout_sec=60.0))
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=20.0))
        future = self.start_client.call_async(Trigger.Request())
        self.assertTrue(self.wait(future.done, 10.0))
        self.assertTrue(future.result().success)

        # 4-8: capture initial path, pallet detection/costmap insertion and replan.
        self.assertTrue(self.wait(lambda: bool(self.paths), 60.0))
        initial = self.signature(self.paths[-1])
        self.assertTrue(self.wait(
            lambda: any('spawn:blocking_pallet' in e.data for e in self.events), 90.0))
        self.assertTrue(self.wait(lambda: any(m.detections for m in self.fused), 10.0))
        self.assertTrue(self.wait(lambda: any(m.width > 0 for m in self.obstacles), 10.0))
        self.assertTrue(self.wait(
            lambda: any(self.signature(path) != initial for path in self.paths), 60.0))

        # 9-12: moving worker must cause a stop and later motion resume.
        self.assertTrue(self.wait(
            lambda: any('spawn:crossing_worker' in e.data for e in self.events), 90.0))
        stop_index = len(self.velocities)
        self.assertTrue(self.wait(lambda: any(
            abs(v.linear.x) < 0.02 and abs(v.angular.z) < 0.02
            for v in self.velocities[stop_index:]), 30.0))
        resume_index = len(self.velocities)
        self.assertTrue(self.wait(lambda: any(
            abs(v.linear.x) > 0.05 for v in self.velocities[resume_index:]), 60.0))

        # 13-18: box avoidance, dock, unload, complete and no collision.
        self.assertTrue(self.wait(
            lambda: any('spawn:fallen_box' in e.data for e in self.events), 90.0))
        self.assertTrue(self.wait(lambda: any(s.data == 'DOCKED' for s in self.states), 180.0))
        self.assertTrue(self.wait(lambda: any(s.data == 'UNLOADING' for s in self.states), 10.0))
        self.assertTrue(self.wait(lambda: any(s.data == 'COMPLETED' for s in self.states), 30.0))
        self.assertTrue(self.cargo and self.cargo[-1].data == '')
        self.assertTrue(self.collisions)
        self.assertEqual(max(item.data for item in self.collisions), 0)
