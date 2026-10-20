"""Live warehouse acceptance test: mission M01 end to end with perception."""

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
    """Run mission M01 from the HOME dock to S2 with live perception and no collision."""

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
        self.collisions = []
        for message, topic, target in (
                (NavPath, '/plan', self.paths),
                (FusedDetectionArray, '/fusion/detections_3d', self.fused),
                (String, '/mission/state', self.states),
                (String, '/mission/cargo', self.cargo),
                (String, '/benchmark/events', self.events),
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
        # Launch headless, wait for active Nav2, start the missions.
        self.assertTrue(self.navigation.wait_for_server(timeout_sec=60.0))
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=20.0))
        future = self.start_client.call_async(Trigger.Request())
        # The mission state is the evidence: Fast DDS can drop a response
        # sent before the client's reply reader is matched, although the
        # mission has started.
        self.assertTrue(self.wait(lambda: self.reached('ACCEPTED'), 10.0))
        if future.done():
            self.assertTrue(future.result().success)

        # The robot boots docked at HOME: M01 undocks, then drives to pickup.
        self.assertTrue(self.wait(lambda: self.reached('UNDOCKING'), 30.0))
        self.assertTrue(self.wait(lambda: self.reached('GO_TO_PICKUP'), 90.0))

        # The blocking pallet is dropped as M01 leaves HOME; it must reach the
        # costmaps through fusion and the bridge, and the plan must change.
        self.assertTrue(self.wait(lambda: bool(self.paths), 60.0))
        initial = self.signature(self.paths[-1])
        self.assertTrue(self.wait(
            lambda: any('spawn:blocking_pallet' in e.data for e in self.events), 30.0))
        self.assertTrue(self.wait(lambda: any(m.detections for m in self.fused), 30.0))
        self.assertTrue(self.wait(lambda: any(m.width > 0 for m in self.obstacles), 30.0))
        self.assertTrue(self.wait(
            lambda: any(self.signature(path) != initial for path in self.paths), 60.0))

        # Pickup, dock at S2 (validated: 8 cm, 5 deg, stopped), unload, undock.
        # Yielding to the roaming workers is random in time and place; the
        # deterministic stop/resume check is test_canonical_launch_smoke.
        self.assertTrue(self.wait(lambda: self.reached('DOCKED'), 300.0))
        self.assertTrue(self.wait(lambda: self.reached('UNLOADING'), 30.0))
        self.assertTrue(self.wait(lambda: self.reached('COMPLETED'), 90.0))
        self.assertTrue(self.cargo and self.cargo[-1].data == '')
        self.assertTrue(self.collisions)
        self.assertEqual(max(item.data for item in self.collisions), 0)

    def reached(self, state):
        return any(s.data == state for s in self.states)
