"""Fault handling contracts for timestamps, sensors, assets and calibration."""

from contracts import assert_contains, text


def test_stale_future_and_invalid_frames_are_counted():
    bridge = text('navigation_bridge',
                  'src/detection_obstacle_bridge_node.cpp')
    assert_contains(bridge, 'rejected_stale_', 'rejected_future_',
                    'rejected_zero_stamp_', 'rejected_empty_frame_',
                    'rejected_missing_tf')


def test_missing_sensor_data_has_bounded_timeout_and_clearing():
    bridge = text('navigation_bridge',
                  'src/detection_obstacle_bridge_node.cpp')
    sync = text('perception_core', 'src/nodes/sensor_sync_node.cpp')
    assert_contains(bridge, 'obstacle_timeout_', 'clearing_publisher_',
                    'makeCloud({}, now())')
    assert_contains(sync, 'dropped_messages', 'sync_tolerance_ms')


def test_missing_model_dataset_and_calibration_fail_explicitly():
    player = text('kitti_ros2_player',
                  'kitti_ros2_player/kitti_player_node.py')
    detector = text('yolo_detector', 'yolo_detector/model_loader.py')
    calibration = text('perception_core',
                       'src/nodes/calibration_monitor_node.cpp')
    assert_contains(player, 'dataset', 'raise')
    assert_contains(detector, 'is_file', 'YOLO model not found', 'RuntimeError')
    assert_contains(calibration, 'translation', 'rotation', 'projection')
