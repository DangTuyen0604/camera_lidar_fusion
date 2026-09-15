from pathlib import Path
import time

from cv_bridge import CvBridge
from fusion_interfaces.msg import Detection2DArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from yolo_detector.detection_converter import (
    detection_array_to_message,
)
from yolo_detector.model_loader import YoloOnnxDetector
from yolo_detector.visualization import draw_detections


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


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', str(_default_model_path()))
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_topic', '/kitti/camera/image_raw')
        self.declare_parameter('detections_topic', '/detections_2d')
        self.declare_parameter('debug_image_topic', '/yolo/debug/image')

        model_path = self.get_parameter('model_path').value
        confidence_threshold = self.get_parameter(
            'confidence_threshold'
        ).value
        iou_threshold = self.get_parameter('iou_threshold').value
        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.detector = YoloOnnxDetector(
            model_path,
            confidence_threshold=confidence_threshold,
            iou_threshold=iou_threshold,
        )
        self.bridge = CvBridge()
        self.frame_count = 0
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            detections_topic,
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            debug_image_topic,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
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
        detection_message = detection_array_to_message(
            detections,
            image_message.header,
            inference_ms,
        )
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


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
