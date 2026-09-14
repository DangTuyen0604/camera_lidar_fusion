from builtin_interfaces.msg import Time
from kitti_ros2_player.calibration_parser import LidarCameraCalibration
from pathlib import Path

import cv2
from kitti_ros2_player.image_loader import (
    ImageLoader,
    load_kitti_timestamps,
    parse_kitti_timestamp,
    TimestampSequence,
)
from kitti_ros2_player.pointcloud_loader import (
    create_pointcloud2,
    draw_lidar_overlay,
    pointcloud2_to_array,
    PointCloudLoader,
    project_velodyne_to_image,
)
import numpy as np
import pytest
from sensor_msgs.msg import PointField


def write_image(path, value):
    image = np.full((2, 3, 3), value, dtype=np.uint8)
    assert cv2.imwrite(str(path), image)


def test_images_are_sorted_and_loop(tmp_path):
    write_image(tmp_path / '0000000002.png', 2)
    write_image(tmp_path / '0000000000.png', 0)
    write_image(tmp_path / '0000000001.png', 1)

    loader = ImageLoader(tmp_path, loop=True)

    paths = [loader.next_image()[1].name for _ in range(4)]

    assert paths == [
        '0000000000.png',
        '0000000001.png',
        '0000000002.png',
        '0000000000.png',
    ]


def test_non_looping_loader_stops_after_last_image(tmp_path):
    write_image(tmp_path / '0000000000.png', 0)

    loader = ImageLoader(tmp_path, loop=False)

    _, path = loader.next_image()
    assert path.name == '0000000000.png'

    with pytest.raises(StopIteration):
        loader.next_image()


def test_loader_rejects_empty_directory(tmp_path):
    with pytest.raises(RuntimeError, match='No KITTI PNG images'):
        ImageLoader(tmp_path)


def test_loader_rejects_missing_directory(tmp_path):
    missing_directory = Path(tmp_path) / 'missing'

    with pytest.raises(RuntimeError, match='directory not found'):
        ImageLoader(missing_directory)


def write_pointcloud(path, points):
    np.asarray(points, dtype='<f4').tofile(path)


def test_pointcloud_loader_and_message_conversion(tmp_path):
    points = np.asarray([[1.0, 2.0, 3.0, 0.5]], dtype=np.float32)
    write_pointcloud(tmp_path / '0000000001.bin', points)
    write_pointcloud(tmp_path / '0000000000.bin', points)
    loader = PointCloudLoader(tmp_path, loop=True)
    names = [loader.next_pointcloud()[1].name for _ in range(3)]
    assert names == [
        '0000000000.bin',
        '0000000001.bin',
        '0000000000.bin',
    ]

    stamp = Time(sec=12, nanosec=34)
    message = create_pointcloud2(points, stamp)
    assert message.header.stamp == stamp
    assert message.header.frame_id == 'velodyne'
    assert message.point_step == 16
    assert all(
        field.datatype == PointField.FLOAT32 for field in message.fields
    )
    np.testing.assert_allclose(pointcloud2_to_array(message), points)


def test_timestamp_parsing_and_rebase(tmp_path):
    first = parse_kitti_timestamp('2011-09-26 13:04:32.345808896')
    second = parse_kitti_timestamp('2011-09-26 13:04:32.449188864')
    assert second - first == 103_379_968

    image_path = tmp_path / 'image.txt'
    lidar_path = tmp_path / 'lidar.txt'
    image_path.write_text(
        '2011-09-26 13:04:32.345808896\n'
        '2011-09-26 13:04:32.445808896\n',
        encoding='utf-8',
    )
    lidar_path.write_text(
        '2011-09-26 13:04:32.335808896\n'
        '2011-09-26 13:04:32.435808896\n',
        encoding='utf-8',
    )
    sequence = TimestampSequence.load(image_path, lidar_path)
    assert len(sequence) == 2
    assert sequence.max_absolute_sensor_offset_ns == 10_000_000
    assert sequence.cycle_duration_ns == 200_000_000
    assert sequence.rebased_timestamp_ns(1, 0, 1_000) == 100_001_000

    image_path.write_text(
        '2011-09-26 13:04:32.445808896\n'
        '2011-09-26 13:04:32.345808896\n',
        encoding='utf-8',
    )
    with pytest.raises(RuntimeError, match='strictly increasing'):
        load_kitti_timestamps(image_path)


def test_projection_and_overlay():
    calibration = LidarCameraCalibration(
        tr_velo_to_cam=np.eye(4),
        r_rect_00=np.eye(4),
        p_rect_02=np.asarray([
            [100.0, 0.0, 50.0, 0.0],
            [0.0, 100.0, 40.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]),
    )
    points = np.asarray([
        [0.0, 0.0, 10.0, 0.5],
        [1.0, 2.0, 10.0, 0.7],
        [0.0, 0.0, -1.0, 0.9],
    ])
    pixels, depths, intensities = project_velodyne_to_image(
        points,
        calibration,
        image_width=100,
        image_height=80,
    )
    np.testing.assert_allclose(pixels, [[50.0, 40.0], [60.0, 60.0]])
    np.testing.assert_allclose(depths, [10.0, 10.0])
    np.testing.assert_allclose(intensities, [0.5, 0.7])
    overlay = draw_lidar_overlay(
        np.zeros((80, 100, 3), dtype=np.uint8),
        pixels,
        depths,
    )
    assert np.any(overlay != 0)
