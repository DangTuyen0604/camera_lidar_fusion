"""
Live camera window with optional detection boxes and frame capture.

Keys (window focused):
  s  save the current raw frame (no overlay) as PNG for later labelling
  d  toggle the /detections_2d boxes
  q  close the window
"""

from pathlib import Path

import cv2
from cv_bridge import CvBridge
from fusion_interfaces.msg import Detection2DArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from yolo_detector.detection_converter import detection_from_message
from yolo_detector.visualization import draw_detections


def frame_filename(stamp):
    """Sortable, unique PNG name from a ROS header stamp."""
    return f'frame_{stamp.sec:010d}_{stamp.nanosec:09d}.png'


class CameraViewer(Node):
    """Keep the latest image and detections for the GUI loop in main()."""

    def __init__(self):
        super().__init__('camera_viewer')
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        detections_topic = self.declare_parameter(
            'detections_topic', '/detections_2d').value
        self.show_detections = bool(self.declare_parameter('show_detections', True).value)
        self.save_directory = Path(self.declare_parameter(
            'save_directory', str(Path.home() / 'camera_dataset')).value).expanduser()
        self.window = self.declare_parameter('window_name', 'AMR camera').value
        self.bridge = CvBridge()
        self.image = None
        self.stamp = None
        self.detections = []
        self.saved = 0
        self.create_subscription(Image, image_topic, self._image, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, detections_topic, self._detections, 10)
        self.get_logger().info(
            f'Showing {image_topic}; press s to save frames to {self.save_directory}, '
            'd to toggle detections, q to close')

    def _image(self, message):
        self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.stamp = message.header.stamp

    def _detections(self, message):
        self.detections = [detection_from_message(item) for item in message.detections]

    def frame(self):
        """Image to display, or None before the first frame arrives."""
        if self.image is None:
            return None
        output = draw_detections(self.image, self.detections) \
            if self.show_detections else self.image.copy()
        hint = f's: save ({self.saved})   d: boxes   q: close'
        cv2.putText(output, hint, (8, output.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return output

    def save(self):
        """Write the raw frame, without overlay, for dataset labelling."""
        if self.image is None:
            return
        self.save_directory.mkdir(parents=True, exist_ok=True)
        path = self.save_directory / frame_filename(self.stamp)
        cv2.imwrite(str(path), self.image)
        self.saved += 1
        self.get_logger().info(f'Saved {path}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    cv2.namedWindow(node.window, cv2.WINDOW_NORMAL)
    sized = False
    try:
        # OpenCV windows must be driven from the main thread.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            frame = node.frame()
            if frame is not None:
                if not sized:  # Open at the sensor resolution; still resizable.
                    cv2.resizeWindow(node.window, frame.shape[1], frame.shape[0])
                    sized = True
                cv2.imshow(node.window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                node.save()
            elif key == ord('d'):
                node.show_detections = not node.show_detections
            elif key == ord('q') or cv2.getWindowProperty(
                    node.window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
