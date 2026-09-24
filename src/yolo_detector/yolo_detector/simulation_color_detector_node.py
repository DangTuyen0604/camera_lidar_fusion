"""Detect the orange simulated worker from live Gazebo camera frames."""

import time

from cv_bridge import CvBridge
from fusion_interfaces.msg import Detection2DArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from yolo_detector.color_detector import detect_color_blobs
from yolo_detector.detection_converter import detection_array_to_message


class SimulationColorDetectorNode(Node):
    """Publish Detection2DArray messages derived from live image pixels."""

    def __init__(self):
        super().__init__('simulation_color_detector_node')
        image_topic = self.declare_parameter(
            'image_topic', '/fusion/synced/image').value
        detections_topic = self.declare_parameter(
            'detections_topic', '/detections_2d').value
        self.lower_hsv = tuple(self.declare_parameter(
            'lower_hsv', [3, 80, 60]).value)
        self.upper_hsv = tuple(self.declare_parameter(
            'upper_hsv', [35, 255, 255]).value)
        self.min_area = int(self.declare_parameter('min_area', 150).value)
        self.padding = int(self.declare_parameter('bbox_padding', 4).value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Detection2DArray, detections_topic, 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Simulation color detector ready on {image_topic}')

    def image_callback(self, message):
        started = time.perf_counter()
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        detections = detect_color_blobs(
            image, self.lower_hsv, self.upper_hsv,
            min_area=self.min_area, padding=self.padding)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.publisher.publish(detection_array_to_message(
            detections, message.header, elapsed_ms))


def main(args=None):
    rclpy.init(args=args)
    node = SimulationColorDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # rclpy can surface a take_message conversion error while launch is
        # concurrently tearing down the context. Preserve genuine runtime
        # failures, but make normal launch shutdown idempotent.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
