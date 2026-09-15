from builtin_interfaces.msg import Time
from kitti_ros2_player.pointcloud_loader import ProjectedPointCloud
import numpy as np
from std_msgs.msg import Header
from yolo_detector.detection_converter import (
    Detection,
    detection_array_to_message,
    detection_from_dict,
    detection_from_message,
    detection_to_message,
    fuse_detections,
)


def test_detection_json_round_trip():
    detection = Detection(
        class_id=2,
        label='car',
        confidence=0.91234567,
        bbox=(10, 20, 110, 120),
    )

    restored = detection_from_dict(detection.to_dict())

    assert restored.class_id == 2
    assert restored.label == 'car'
    assert restored.confidence == 0.912346
    assert restored.bbox == (10, 20, 110, 120)


def test_detection_ros_message_round_trip():
    detection = Detection(2, 'car', 0.91, (10, 20, 110, 120))

    message = detection_to_message(detection)
    restored = detection_from_message(message)

    assert message.class_id == 2
    assert message.class_name == 'car'
    assert np.isclose(message.confidence, 0.91)
    assert restored.class_id == detection.class_id
    assert restored.label == detection.label
    assert restored.bbox == detection.bbox
    assert all(isinstance(coordinate, int) for coordinate in restored.bbox)


def test_detection_message_rounds_pixel_coordinates_for_opencv():
    message = detection_to_message(
        Detection(2, 'car', 0.91, (10, 20, 110, 120))
    )
    message.x_min = 10.4
    message.y_min = 20.6
    message.x_max = 109.6
    message.y_max = 120.4

    restored = detection_from_message(message)

    assert restored.bbox == (10, 21, 110, 120)
    assert all(isinstance(coordinate, int) for coordinate in restored.bbox)


def test_detection_array_copies_header_and_allows_empty_detections():
    header = Header(
        stamp=Time(sec=123, nanosec=456),
        frame_id='camera_optical_frame',
    )

    message = detection_array_to_message([], header, 4.25)

    assert message.header.stamp == header.stamp
    assert message.header.frame_id == header.frame_id
    assert message.inference_ms == np.float32(4.25)
    assert list(message.detections) == []


def test_fusion_rejects_depth_outlier_and_uses_median():
    detection = Detection(2, 'car', 0.91, (10, 10, 90, 90))
    depths = np.asarray([9.8, 10.0, 10.2, 50.0])
    camera_points = np.column_stack((
        np.asarray([1.0, 1.1, 0.9, 8.0]),
        np.asarray([0.0, 0.1, -0.1, 2.0]),
        depths,
    ))
    projected = ProjectedPointCloud(
        pixels=np.asarray([
            [30.0, 30.0],
            [40.0, 40.0],
            [50.0, 50.0],
            [60.0, 60.0],
        ]),
        depths=depths,
        intensities=np.ones(4),
        camera_points=camera_points,
        velodyne_points=np.zeros((4, 3)),
    )
    fused = fuse_detections([detection], projected)
    assert len(fused) == 1
    assert fused[0].distance_m == 10.0
    assert fused[0].point_count == 3
    assert fused[0].raw_point_count == 4
