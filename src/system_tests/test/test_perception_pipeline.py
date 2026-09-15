import struct
import time
import unittest

from builtin_interfaces.msg import Time
from fusion_interfaces.msg import Detection2D
from fusion_interfaces.msg import Detection2DArray
from fusion_interfaces.msg import FusedDetectionArray
from fusion_interfaces.msg import SyncStatus
import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='perception_core',
            executable='sensor_sync_node',
            parameters=[{'sync_tolerance_ms': 50.0}],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='perception_core',
            executable='object_fusion_node',
            parameters=[{
                'sync_tolerance_ms': 50.0,
                'tf_timeout_ms': 100.0,
                'min_depth': 0.1,
            }],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'camera_optical_frame',
                '--child-frame-id', 'velodyne',
            ],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPerceptionPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('perception_pipeline_test')
        self.sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_publisher = self.node.create_publisher(
            Image, '/camera/image', self.sensor_qos
        )
        self.info_publisher = self.node.create_publisher(
            CameraInfo, '/camera/camera_info', self.sensor_qos
        )
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2, '/lidar/points', self.sensor_qos
        )
        self.direct_cloud_publisher = self.node.create_publisher(
            PointCloud2, '/fusion/synced/points', self.sensor_qos
        )
        self.direct_info_publisher = self.node.create_publisher(
            CameraInfo, '/fusion/synced/camera_info', self.sensor_qos
        )
        self.detection_publisher = self.node.create_publisher(
            Detection2DArray, '/detections_2d', 10
        )

        self.synced_images = []
        self.synced_clouds = []
        self.sync_statuses = []
        self.fused_arrays = []
        self.marker_arrays = []
        self.annotated_images = []
        self.node.create_subscription(
            Image,
            '/fusion/synced/image',
            self.synced_images.append,
            self.sensor_qos,
        )
        self.node.create_subscription(
            PointCloud2,
            '/fusion/synced/points',
            self.synced_clouds.append,
            self.sensor_qos,
        )
        self.node.create_subscription(
            SyncStatus, '/fusion/sync_status', self.sync_statuses.append, 10
        )
        self.node.create_subscription(
            FusedDetectionArray,
            '/fusion/detections_3d',
            self.fused_arrays.append,
            10,
        )
        self.node.create_subscription(
            MarkerArray,
            '/fusion/object_markers',
            self.marker_arrays.append,
            10,
        )
        self.node.create_subscription(
            Image,
            '/fusion/annotated_image',
            self.annotated_images.append,
            self.sensor_qos,
        )
        self.assertTrue(self.wait_for(self.publishers_are_connected, 10.0))
        time.sleep(1.0)

    def tearDown(self):
        self.node.destroy_node()

    def wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publishers_are_connected(self):
        return (
            self.image_publisher.get_subscription_count() > 0
            and self.info_publisher.get_subscription_count() > 0
            and self.cloud_publisher.get_subscription_count() > 0
            and self.detection_publisher.get_subscription_count() > 0
        )

    @staticmethod
    def header_stamp(second):
        return Time(sec=second, nanosec=0)

    def make_image(self, second):
        message = Image()
        message.header.stamp = self.header_stamp(second)
        message.header.frame_id = 'camera_optical_frame'
        message.height = 80
        message.width = 100
        message.encoding = 'bgr8'
        message.step = 300
        message.data = bytes(message.step * message.height)
        return message

    def make_camera_info(self, second):
        message = CameraInfo()
        message.header.stamp = self.header_stamp(second)
        message.header.frame_id = 'camera_optical_frame'
        message.height = 80
        message.width = 100
        message.p = [
            100.0, 0.0, 50.0, 0.0,
            0.0, 100.0, 40.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def make_cloud(self, second, frame='velodyne', points=None):
        if points is None:
            points = [
                (0.0, 0.0, 10.0, 1.0),
                (0.1, 0.0, 10.1, 1.0),
                (-0.1, 0.0, 9.9, 1.0),
            ]
        message = PointCloud2()
        message.header.stamp = self.header_stamp(second)
        message.header.frame_id = frame
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity', offset=12,
                datatype=PointField.FLOAT32, count=1,
            ),
        ]
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.data = b''.join(struct.pack('<ffff', *point) for point in points)
        message.is_dense = True
        return message

    def make_detections(self, second, with_detection=True):
        message = Detection2DArray()
        message.header.stamp = self.header_stamp(second)
        message.header.frame_id = 'camera_optical_frame'
        message.inference_ms = 5.0
        if with_detection:
            detection = Detection2D()
            detection.class_id = 2
            detection.class_name = 'car'
            detection.confidence = 0.91
            detection.x_min = 40.0
            detection.y_min = 30.0
            detection.x_max = 60.0
            detection.y_max = 50.0
            message.detections = [detection]
        return message

    def publish_raw_frame(self, second, frame='velodyne', points=None):
        expected_status_count = len(self.sync_statuses) + 1
        for _ in range(3):
            self.image_publisher.publish(self.make_image(second))
            self.info_publisher.publish(self.make_camera_info(second))
            self.cloud_publisher.publish(
                self.make_cloud(second, frame=frame, points=points)
            )
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.wait_for(
            lambda: len(self.sync_statuses) >= expected_status_count
        ))

    def publish_detection_and_wait(self, message):
        expected_count = len(self.fused_arrays) + 1
        self.detection_publisher.publish(message)
        self.assertTrue(self.wait_for(
            lambda: len(self.fused_arrays) >= expected_count
        ))
        return self.fused_arrays[-1]

    def test_synchronization_and_typed_fusion_edge_cases(self):
        topic_types = dict(self.node.get_topic_names_and_types())
        self.assertIn('fusion_interfaces/msg/Detection2DArray', topic_types['/detections_2d'])
        self.assertIn(
            'fusion_interfaces/msg/FusedDetectionArray',
            topic_types['/fusion/detections_3d'],
        )
        self.assertIn('sensor_msgs/msg/PointCloud2', topic_types['/fusion/synced/points'])

        self.publish_raw_frame(100)
        self.assertTrue(self.sync_statuses[-1].healthy)
        self.assertEqual(self.sync_statuses[-1].camera_lidar_offset_ms, 0.0)
        self.assertTrue(any(msg.header.stamp.sec == 100 for msg in self.synced_images))
        self.assertTrue(any(msg.header.stamp.sec == 100 for msg in self.synced_clouds))

        valid = self.publish_detection_and_wait(self.make_detections(100))
        self.assertEqual(valid.header.stamp.sec, 100)
        self.assertEqual(valid.header.frame_id, 'camera_optical_frame')
        self.assertEqual(len(valid.detections), 1)
        self.assertTrue(valid.detections[0].valid)
        self.assertAlmostEqual(valid.detections[0].depth, 10.0, places=4)
        self.assertEqual(valid.detections[0].lidar_point_count, 3)
        self.assertTrue(self.wait_for(lambda: bool(self.marker_arrays)))
        self.assertTrue(self.wait_for(lambda: bool(self.annotated_images)))
        self.assertEqual(self.annotated_images[-1].header.stamp.sec, 100)
        added_markers = [
            marker for marker in self.marker_arrays[-1].markers
            if marker.action == marker.ADD
        ]
        self.assertTrue(added_markers)
        self.assertGreater(added_markers[0].color.g, added_markers[0].color.r)
        self.assertAlmostEqual(
            added_markers[0].lifetime.nanosec / 1.0e9,
            0.35,
            places=2,
        )

        empty = self.publish_detection_and_wait(
            self.make_detections(100, with_detection=False)
        )
        self.assertEqual(empty.detections, [])

        self.publish_raw_frame(101, points=[])
        empty_cloud = self.publish_detection_and_wait(self.make_detections(101))
        self.assertEqual(len(empty_cloud.detections), 1)
        self.assertFalse(empty_cloud.detections[0].valid)

        self.publish_raw_frame(102, frame='wrong_lidar_frame')
        wrong_frame = self.publish_detection_and_wait(self.make_detections(102))
        self.assertFalse(wrong_frame.detections[0].valid)

        self.direct_info_publisher.publish(self.make_camera_info(104))
        self.direct_cloud_publisher.publish(self.make_cloud(103))
        time.sleep(0.2)
        wrong_time = self.publish_detection_and_wait(self.make_detections(104))
        self.assertFalse(wrong_time.detections[0].valid)
