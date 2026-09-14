from dataclasses import dataclass
from pathlib import Path

import numpy as np
from sensor_msgs.msg import CameraInfo
import yaml


@dataclass(frozen=True)
class LidarCameraCalibration:
    tr_velo_to_cam: np.ndarray
    r_rect_00: np.ndarray
    p_rect_02: np.ndarray


def matrix_to_quaternion(rotation):
    rotation = np.asarray(rotation, dtype=np.float64)

    if rotation.shape != (3, 3):
        raise RuntimeError(
            f'Expected rotation matrix shape (3, 3), got {rotation.shape}'
        )

    trace = np.trace(rotation)

    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        diagonal_index = int(np.argmax(np.diag(rotation)))

        if diagonal_index == 0:
            scale = np.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            quaternion = np.asarray([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif diagonal_index == 1:
            scale = np.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            quaternion = np.asarray([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = np.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            quaternion = np.asarray([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])

    return quaternion / np.linalg.norm(quaternion)


def quaternion_to_matrix(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)

    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise RuntimeError(
            'Quaternion must contain four finite values in x, y, z, w order'
        )

    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise RuntimeError('Quaternion norm must be greater than zero')

    x, y, z, w = quaternion / norm
    return np.asarray([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])


def read_yaml_file(yaml_path):
    yaml_path = Path(yaml_path).expanduser()

    if not yaml_path.is_file():
        raise RuntimeError(f'Calibration YAML not found: {yaml_path}')

    try:
        configuration = yaml.safe_load(
            yaml_path.read_text(encoding='utf-8')
        )
    except yaml.YAMLError as error:
        raise RuntimeError(
            f'Invalid calibration YAML: {yaml_path}: {error}'
        ) from error

    if not isinstance(configuration, dict):
        raise RuntimeError(
            f'Calibration YAML must contain a mapping: {yaml_path}'
        )

    return configuration


def _read_yaml_matrix(configuration, key, rows, cols):
    section = configuration.get(key)

    if not isinstance(section, dict):
        raise RuntimeError(f'Invalid or missing YAML matrix: {key}')

    if section.get('rows') != rows or section.get('cols') != cols:
        raise RuntimeError(
            f'{key} must have shape {rows}x{cols}'
        )

    data = np.asarray(section.get('data'), dtype=np.float64)
    if data.size != rows * cols or not np.isfinite(data).all():
        raise RuntimeError(
            f'{key} must contain {rows * cols} finite values'
        )

    return data.reshape(rows, cols)


def load_camera_info_yaml(intrinsics_path):
    configuration = read_yaml_file(intrinsics_path)
    width = configuration.get('image_width')
    height = configuration.get('image_height')

    if not isinstance(width, int) or width <= 0:
        raise RuntimeError(f'Invalid YAML image_width: {width}')

    if not isinstance(height, int) or height <= 0:
        raise RuntimeError(f'Invalid YAML image_height: {height}')

    camera_matrix = _read_yaml_matrix(
        configuration,
        'camera_matrix',
        3,
        3,
    )
    distortion = _read_yaml_matrix(
        configuration,
        'distortion_coefficients',
        1,
        5,
    )
    rectification = _read_yaml_matrix(
        configuration,
        'rectification_matrix',
        3,
        3,
    )
    projection = _read_yaml_matrix(
        configuration,
        'projection_matrix',
        3,
        4,
    )

    message = CameraInfo()
    message.width = width
    message.height = height
    message.distortion_model = configuration.get(
        'distortion_model',
        'plumb_bob',
    )
    message.d = distortion.reshape(-1).tolist()
    message.k = camera_matrix.reshape(-1).tolist()
    message.r = rectification.reshape(-1).tolist()
    message.p = projection.reshape(-1).tolist()
    return message


def load_lidar_camera_calibration_yaml(
    intrinsics_path,
    extrinsics_path,
    expected_parent_frame='velodyne',
    expected_child_frame='camera_optical_frame',
):
    intrinsics = read_yaml_file(intrinsics_path)
    extrinsics = read_yaml_file(extrinsics_path)

    parent_frame = extrinsics.get('parent_frame')
    child_frame = extrinsics.get('child_frame')
    convention = extrinsics.get('convention')

    if parent_frame != expected_parent_frame:
        raise RuntimeError(
            'Extrinsic parent frame mismatch: '
            f'expected={expected_parent_frame}, actual={parent_frame}'
        )

    if child_frame != expected_child_frame:
        raise RuntimeError(
            'Extrinsic child frame mismatch: '
            f'expected={expected_child_frame}, actual={child_frame}'
        )

    if convention != 'child_pose_in_parent':
        raise RuntimeError(
            'Extrinsic convention must be child_pose_in_parent'
        )

    translation_config = extrinsics.get('translation')
    rotation_config = extrinsics.get('rotation')
    if not isinstance(translation_config, dict):
        raise RuntimeError('Invalid or missing extrinsic translation')
    if not isinstance(rotation_config, dict):
        raise RuntimeError('Invalid or missing extrinsic rotation')

    try:
        translation = np.asarray([
            translation_config['x'],
            translation_config['y'],
            translation_config['z'],
        ], dtype=np.float64)
        quaternion = np.asarray([
            rotation_config['x'],
            rotation_config['y'],
            rotation_config['z'],
            rotation_config['w'],
        ], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            'Extrinsic translation/rotation contains invalid values'
        ) from error

    if not np.isfinite(translation).all():
        raise RuntimeError('Extrinsic translation must contain finite values')

    # The YAML follows TF2: it stores the pose of the child camera frame in
    # the parent Velodyne frame. Projection needs the opposite direction.
    lidar_from_camera = np.eye(4, dtype=np.float64)
    lidar_from_camera[:3, :3] = quaternion_to_matrix(quaternion)
    lidar_from_camera[:3, 3] = translation
    camera_from_lidar = np.linalg.inv(lidar_from_camera)
    projection = _read_yaml_matrix(
        intrinsics,
        'projection_matrix',
        3,
        4,
    )

    return LidarCameraCalibration(
        tr_velo_to_cam=camera_from_lidar,
        r_rect_00=np.eye(4, dtype=np.float64),
        p_rect_02=projection,
    )


def read_calibration_file(calibration_path):
    calibration_path = Path(calibration_path).expanduser()

    if not calibration_path.is_file():
        raise RuntimeError(
            f'Calibration file not found: {calibration_path}'
        )

    calibration = {}

    for line in calibration_path.read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue

        key, raw_values = line.split(':', 1)

        try:
            calibration[key] = [
                float(value) for value in raw_values.split()
            ]
        except ValueError:
            # Fields such as calib_time contain text and are not matrices.
            continue

    return calibration


def load_camera_info(calibration_path):
    calibration = read_calibration_file(calibration_path)
    image_size = calibration.get('S_rect_02')
    projection = calibration.get('P_rect_02')

    if image_size is None or len(image_size) != 2:
        raise RuntimeError('Invalid or missing S_rect_02 calibration')

    if projection is None or len(projection) != 12:
        raise RuntimeError('Invalid or missing P_rect_02 calibration')

    width = int(image_size[0])
    height = int(image_size[1])

    if width <= 0 or height <= 0:
        raise RuntimeError(
            f'Invalid rectified image size: {width}x{height}'
        )

    message = CameraInfo()
    message.width = width
    message.height = height
    message.distortion_model = 'plumb_bob'

    # The KITTI sync images are already rectified, so downstream nodes must
    # not apply lens distortion or rectification a second time.
    message.d = [0.0] * 5
    message.k = [
        projection[0], projection[1], projection[2],
        projection[4], projection[5], projection[6],
        projection[8], projection[9], projection[10],
    ]
    message.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    message.p = projection

    return message


def load_lidar_camera_calibration(
    camera_calibration_path,
    velodyne_calibration_path,
):
    camera_calibration = read_calibration_file(camera_calibration_path)
    velodyne_calibration = read_calibration_file(
        velodyne_calibration_path
    )

    rectification = camera_calibration.get('R_rect_00')
    projection = camera_calibration.get('P_rect_02')
    rotation = velodyne_calibration.get('R')
    translation = velodyne_calibration.get('T')

    if rectification is None or len(rectification) != 9:
        raise RuntimeError('Invalid or missing R_rect_00 calibration')

    if projection is None or len(projection) != 12:
        raise RuntimeError('Invalid or missing P_rect_02 calibration')

    if rotation is None or len(rotation) != 9:
        raise RuntimeError('Invalid or missing Velodyne R calibration')

    if translation is None or len(translation) != 3:
        raise RuntimeError('Invalid or missing Velodyne T calibration')

    tr_velo_to_cam = np.eye(4, dtype=np.float64)
    tr_velo_to_cam[:3, :3] = np.asarray(
        rotation,
        dtype=np.float64,
    ).reshape(3, 3)
    tr_velo_to_cam[:3, 3] = np.asarray(
        translation,
        dtype=np.float64,
    )

    r_rect_00 = np.eye(4, dtype=np.float64)
    r_rect_00[:3, :3] = np.asarray(
        rectification,
        dtype=np.float64,
    ).reshape(3, 3)

    p_rect_02 = np.asarray(
        projection,
        dtype=np.float64,
    ).reshape(3, 4)

    return LidarCameraCalibration(
        tr_velo_to_cam=tr_velo_to_cam,
        r_rect_00=r_rect_00,
        p_rect_02=p_rect_02,
    )
