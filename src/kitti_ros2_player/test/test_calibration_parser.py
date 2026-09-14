from kitti_ros2_player.calibration_parser import (
    load_camera_info,
    load_camera_info_yaml,
    load_lidar_camera_calibration,
    load_lidar_camera_calibration_yaml,
    matrix_to_quaternion,
)
import numpy as np
import pytest
import yaml


def write_calibration(path, size='1242 375', projection=None):
    if projection is None:
        projection = (
            '721.5377 0.0 609.5593 44.85728 '
            '0.0 721.5377 172.8540 0.2163791 '
            '0.0 0.0 1.0 0.002745884'
        )

    path.write_text(
        'calib_time: 09-Jan-2012 13:57:47\n'
        f'S_rect_02: {size}\n'
        f'P_rect_02: {projection}\n',
        encoding='utf-8',
    )


def test_load_rectified_camera_info(tmp_path):
    calibration_path = tmp_path / 'calib_cam_to_cam.txt'
    write_calibration(calibration_path)

    message = load_camera_info(calibration_path)

    assert message.width == 1242
    assert message.height == 375
    assert message.distortion_model == 'plumb_bob'
    assert list(message.d) == [0.0] * 5
    assert list(message.k) == [
        721.5377, 0.0, 609.5593,
        0.0, 721.5377, 172.8540,
        0.0, 0.0, 1.0,
    ]
    assert list(message.r) == [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    assert len(message.p) == 12


def test_rejects_missing_projection_matrix(tmp_path):
    calibration_path = tmp_path / 'calib_cam_to_cam.txt'
    calibration_path.write_text(
        'S_rect_02: 1242 375\n',
        encoding='utf-8',
    )

    with pytest.raises(RuntimeError, match='P_rect_02'):
        load_camera_info(calibration_path)


def test_rejects_missing_calibration_file(tmp_path):
    with pytest.raises(RuntimeError, match='Calibration file not found'):
        load_camera_info(tmp_path / 'missing.txt')


def test_load_lidar_camera_calibration(tmp_path):
    camera_path = tmp_path / 'calib_cam_to_cam.txt'
    velodyne_path = tmp_path / 'calib_velo_to_cam.txt'
    write_calibration(camera_path)

    with camera_path.open('a', encoding='utf-8') as camera_file:
        camera_file.write(
            'R_rect_00: 1 0 0 0 1 0 0 0 1\n'
        )

    velodyne_path.write_text(
        'R: 1 0 0 0 1 0 0 0 1\n'
        'T: 1 2 3\n',
        encoding='utf-8',
    )

    calibration = load_lidar_camera_calibration(
        camera_path,
        velodyne_path,
    )

    assert calibration.tr_velo_to_cam.shape == (4, 4)
    assert calibration.r_rect_00.shape == (4, 4)
    assert calibration.p_rect_02.shape == (3, 4)
    assert list(calibration.tr_velo_to_cam[:3, 3]) == [1.0, 2.0, 3.0]


def test_matrix_to_quaternion_identity():
    quaternion = matrix_to_quaternion(np.eye(3))

    np.testing.assert_allclose(quaternion, [0.0, 0.0, 0.0, 1.0])


def write_yaml_calibration(intrinsics_path, extrinsics_path):
    intrinsics_path.write_text(
        yaml.safe_dump({
            'image_width': 1242,
            'image_height': 375,
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [721.0, 0.0, 609.0,
                         0.0, 721.0, 172.0,
                         0.0, 0.0, 1.0],
            },
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': [0.0] * 5,
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0],
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [721.0, 0.0, 609.0, 0.0,
                         0.0, 721.0, 172.0, 0.0,
                         0.0, 0.0, 1.0, 0.0],
            },
        }),
        encoding='utf-8',
    )
    extrinsics_path.write_text(
        yaml.safe_dump({
            'parent_frame': 'velodyne',
            'child_frame': 'camera_optical_frame',
            'convention': 'child_pose_in_parent',
            'translation': {'x': 1.0, 'y': 2.0, 'z': 3.0},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        }),
        encoding='utf-8',
    )


def test_load_yaml_camera_info_and_tf_extrinsic(tmp_path):
    intrinsics_path = tmp_path / 'intrinsics.yaml'
    extrinsics_path = tmp_path / 'extrinsics.yaml'
    write_yaml_calibration(intrinsics_path, extrinsics_path)

    camera_info = load_camera_info_yaml(intrinsics_path)
    calibration = load_lidar_camera_calibration_yaml(
        intrinsics_path,
        extrinsics_path,
    )

    assert camera_info.width == 1242
    assert camera_info.height == 375
    assert list(camera_info.k) == [
        721.0, 0.0, 609.0,
        0.0, 721.0, 172.0,
        0.0, 0.0, 1.0,
    ]
    np.testing.assert_allclose(
        calibration.tr_velo_to_cam[:3, 3],
        [-1.0, -2.0, -3.0],
    )
    np.testing.assert_allclose(calibration.r_rect_00, np.eye(4))


def test_yaml_loader_rejects_wrong_frame_direction(tmp_path):
    intrinsics_path = tmp_path / 'intrinsics.yaml'
    extrinsics_path = tmp_path / 'extrinsics.yaml'
    write_yaml_calibration(intrinsics_path, extrinsics_path)
    configuration = yaml.safe_load(
        extrinsics_path.read_text(encoding='utf-8')
    )
    configuration['parent_frame'] = 'camera_optical_frame'
    extrinsics_path.write_text(
        yaml.safe_dump(configuration),
        encoding='utf-8',
    )

    with pytest.raises(RuntimeError, match='parent frame mismatch'):
        load_lidar_camera_calibration_yaml(
            intrinsics_path,
            extrinsics_path,
        )
