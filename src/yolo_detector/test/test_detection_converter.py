from kitti_ros2_player.pointcloud_loader import ProjectedPointCloud
import numpy as np
from yolo_detector.detection_converter import (
    Detection,
    detection_from_dict,
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
