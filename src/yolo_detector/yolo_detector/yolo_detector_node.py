from collections import OrderedDict
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
from kitti_ros2_player.calibration_parser import (
    load_lidar_camera_calibration,
)
from kitti_ros2_player.pointcloud_loader import (
    pointcloud2_to_array,
    project_velodyne_to_image_details,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from yolo_detector.detection_converter import (
    detection_from_dict,
    fuse_detections,
)
from yolo_detector.model_loader import YoloOnnxDetector
from yolo_detector.visualization import draw_detections, draw_fused_detections


def _default_model_path():
    project_root = Path(__file__).resolve().parents[3]
    candidates = [
        Path.cwd() / 'models' / 'yolov8n-opencv.onnx',
        project_root / 'models' / 'yolov8n-opencv.onnx',
    ]

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    return candidates[0]


def _default_data_path(filename):
    project_root = Path(__file__).resolve().parents[3]
    relative_path = Path('data') / 'kitti' / '2011_09_26' / filename
    candidates = [Path.cwd() / relative_path, project_root / relative_path]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def _stamp_key(header):
    return header.stamp.sec, header.stamp.nanosec


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', str(_default_model_path()))
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)

        model_path = self.get_parameter('model_path').value
        confidence_threshold = self.get_parameter(
            'confidence_threshold'
        ).value
        iou_threshold = self.get_parameter('iou_threshold').value
        self.detector = YoloOnnxDetector(
            model_path,
            confidence_threshold=confidence_threshold,
            iou_threshold=iou_threshold,
        )
        self.bridge = CvBridge()
        self.frame_count = 0
        self.detection_publisher = self.create_publisher(
            String,
            '/yolo/detections',
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            '/yolo/debug/image',
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/kitti/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'YOLO ready: {model_path} '
            f'(confidence={confidence_threshold:.2f}, '
            f'iou={iou_threshold:.2f})'
        )

    def image_callback(self, image_message):
        image = self.bridge.imgmsg_to_cv2(
            image_message,
            desired_encoding='bgr8',
        )
        inference_start = time.perf_counter()
        detections = self.detector.detect(image)
        inference_ms = (time.perf_counter() - inference_start) * 1000.0
        payload = {
            'header': {
                'stamp': {
                    'sec': image_message.header.stamp.sec,
                    'nanosec': image_message.header.stamp.nanosec,
                },
                'frame_id': image_message.header.frame_id,
            },
            'inference_ms': round(inference_ms, 3),
            'detections': [
                detection.to_dict() for detection in detections
            ],
        }
        detection_message = String()
        detection_message.data = json.dumps(payload, separators=(',', ':'))
        self.detection_publisher.publish(detection_message)

        if self.debug_publisher.get_subscription_count() > 0:
            debug_image = draw_detections(image, detections)
            debug_message = self.bridge.cv2_to_imgmsg(
                debug_image,
                encoding='bgr8',
            )
            debug_message.header = image_message.header
            self.debug_publisher.publish(debug_message)

        self.frame_count += 1

        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f'YOLO frame: {len(detections)} detections, '
                f'{inference_ms:.1f} ms'
            )


class FusionNode(Node):

    def __init__(self):
        super().__init__('camera_lidar_fusion_node')
        self.declare_parameter(
            'camera_calibration_path',
            str(_default_data_path('calib_cam_to_cam.txt')),
        )
        self.declare_parameter(
            'velodyne_calibration_path',
            str(_default_data_path('calib_velo_to_cam.txt')),
        )
        self.declare_parameter('minimum_lidar_points', 3)
        self.declare_parameter('bbox_shrink', 0.08)
        self.declare_parameter('max_depth', 80.0)
        self.minimum_lidar_points = self.get_parameter(
            'minimum_lidar_points'
        ).value
        self.bbox_shrink = self.get_parameter('bbox_shrink').value
        self.max_depth = self.get_parameter('max_depth').value
        self.calibration = load_lidar_camera_calibration(
            self.get_parameter('camera_calibration_path').value,
            self.get_parameter('velodyne_calibration_path').value,
        )
        self.bridge = CvBridge()
        self.cache_size = 30
        self.images = OrderedDict()
        self.pointclouds = OrderedDict()
        self.pending_detections = OrderedDict()
        self.fused_frame_count = 0
        self.fused_publisher = self.create_publisher(
            String, '/fusion/detections', 10
        )
        self.debug_publisher = self.create_publisher(
            Image, '/fusion/debug/image', qos_profile_sensor_data
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/kitti/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/kitti/velodyne/points',
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.detection_subscription = self.create_subscription(
            String, '/yolo/detections', self.detection_callback, 10
        )
        self.get_logger().info(
            'Fusion ready: exact timestamp cache, '
            f'minimum_lidar_points={self.minimum_lidar_points}'
        )

    def _cache(self, cache, key, value):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.cache_size:
            cache.popitem(last=False)

    def image_callback(self, message):
        key = _stamp_key(message.header)
        image = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding='bgr8',
        ).copy()
        self._cache(self.images, key, (image, message.header))
        self._try_fuse(key)

    def pointcloud_callback(self, message):
        key = _stamp_key(message.header)
        self._cache(self.pointclouds, key, pointcloud2_to_array(message))
        self._try_fuse(key)

    def detection_callback(self, message):
        payload = json.loads(message.data)
        stamp = payload['header']['stamp']
        key = int(stamp['sec']), int(stamp['nanosec'])
        self._cache(self.pending_detections, key, payload)
        self._try_fuse(key)

    def _try_fuse(self, key):
        if not all(key in cache for cache in (
            self.images,
            self.pointclouds,
            self.pending_detections,
        )):
            return
        image, header = self.images.pop(key)
        points = self.pointclouds.pop(key)
        detection_payload = self.pending_detections.pop(key)
        detections = [
            detection_from_dict(data)
            for data in detection_payload['detections']
        ]
        image_height, image_width = image.shape[:2]
        projected = project_velodyne_to_image_details(
            points,
            self.calibration,
            image_width,
            image_height,
            min_depth=0.1,
            max_depth=self.max_depth,
        )
        fused_detections = fuse_detections(
            detections,
            projected,
            minimum_points=self.minimum_lidar_points,
            bbox_shrink=self.bbox_shrink,
        )
        output = {
            'header': detection_payload['header'],
            'detections': [item.to_dict() for item in fused_detections],
            'unfused_detection_count': len(detections) - len(fused_detections),
        }
        output_message = String()
        output_message.data = json.dumps(output, separators=(',', ':'))
        self.fused_publisher.publish(output_message)
        if self.debug_publisher.get_subscription_count() > 0:
            debug_image = draw_fused_detections(
                image,
                detections,
                fused_detections,
            )
            debug_message = self.bridge.cv2_to_imgmsg(
                debug_image,
                encoding='bgr8',
            )
            debug_message.header = header
            self.debug_publisher.publish(debug_message)
        self.fused_frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def fusion_main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
