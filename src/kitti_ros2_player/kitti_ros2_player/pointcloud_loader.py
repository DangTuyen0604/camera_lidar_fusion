from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField


POINT_FIELDS = [
    PointField(
        name='x',
        offset=0,
        datatype=PointField.FLOAT32,
        count=1,
    ),
    PointField(
        name='y',
        offset=4,
        datatype=PointField.FLOAT32,
        count=1,
    ),
    PointField(
        name='z',
        offset=8,
        datatype=PointField.FLOAT32,
        count=1,
    ),
    PointField(
        name='intensity',
        offset=12,
        datatype=PointField.FLOAT32,
        count=1,
    ),
]


class PointCloudLoader:

    def __init__(self, pointcloud_dir, loop=True):
        self.pointcloud_dir = Path(pointcloud_dir).expanduser()
        self.loop = loop

        if not self.pointcloud_dir.is_dir():
            raise RuntimeError(
                'KITTI point cloud directory not found: '
                f'{self.pointcloud_dir}'
            )

        self.pointcloud_paths = sorted(self.pointcloud_dir.glob('*.bin'))

        if not self.pointcloud_paths:
            raise RuntimeError(
                'No KITTI point cloud files found in: '
                f'{self.pointcloud_dir}'
            )

        self.frame_index = 0

    def __len__(self):
        return len(self.pointcloud_paths)

    def next_pointcloud(self):
        if self.frame_index >= len(self.pointcloud_paths):
            if not self.loop:
                raise StopIteration
            self.frame_index = 0

        path = self.pointcloud_paths[self.frame_index]
        raw_points = np.fromfile(path, dtype='<f4')

        if raw_points.size == 0 or raw_points.size % 4 != 0:
            raise RuntimeError(
                f'Invalid KITTI point cloud file: {path} '
                f'({raw_points.size} float32 values)'
            )

        points = raw_points.reshape(-1, 4)
        if not np.isfinite(points).all():
            raise RuntimeError(
                f'KITTI point cloud contains NaN or Inf: {path}'
            )
        self.frame_index += 1

        if (
            self.loop
            and self.frame_index >= len(self.pointcloud_paths)
        ):
            self.frame_index = 0

        return points, path


def create_pointcloud2(points, stamp, frame_id='velodyne'):
    points = np.asarray(points)

    if points.ndim != 2 or points.shape[1] != 4:
        raise RuntimeError(
            f'Expected point cloud shape (N, 4), got {points.shape}'
        )
    if not np.isfinite(points).all():
        raise RuntimeError('Velodyne points must contain finite values')

    points = np.ascontiguousarray(points, dtype='<f4')

    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = points.shape[0]
    message.fields = POINT_FIELDS
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * message.width
    message.data = points.tobytes()
    message.is_dense = bool(np.isfinite(points).all())

    return message


def pointcloud2_to_array(message):
    field_map = {field.name: field for field in message.fields}
    required_fields = ('x', 'y', 'z', 'intensity')

    for field_name in required_fields:
        field = field_map.get(field_name)

        if field is None:
            raise RuntimeError(
                f'PointCloud2 is missing field: {field_name}'
            )

        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise RuntimeError(
                f'PointCloud2 field must be one float32: {field_name}'
            )

    byte_order = '>' if message.is_bigendian else '<'
    point_dtype = np.dtype({
        'names': list(required_fields),
        'formats': [f'{byte_order}f4'] * 4,
        'offsets': [field_map[name].offset for name in required_fields],
        'itemsize': message.point_step,
    })
    point_count = message.width * message.height
    structured_points = np.frombuffer(
        message.data,
        dtype=point_dtype,
        count=point_count,
    )

    return np.column_stack([
        structured_points[name] for name in required_fields
    ]).astype(np.float32, copy=False)


@dataclass(frozen=True)
class ProjectedPointCloud:
    pixels: np.ndarray
    depths: np.ndarray
    intensities: np.ndarray
    camera_points: np.ndarray
    velodyne_points: np.ndarray


def project_velodyne_to_image_details(
    points,
    calibration,
    image_width,
    image_height,
    min_depth=0.1,
    max_depth=None,
):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 4:
        raise RuntimeError(
            f'Expected Velodyne point shape (N, 4), got {points.shape}'
        )
    if image_width <= 0 or image_height <= 0:
        raise RuntimeError(
            f'Invalid image size: {image_width}x{image_height}'
        )
    if min_depth <= 0.0:
        raise RuntimeError(f'min_depth must be positive: {min_depth}')
    if max_depth is not None and max_depth <= min_depth:
        raise RuntimeError(
            f'max_depth must be greater than min_depth: {max_depth}'
        )

    homogeneous_points = np.ones((points.shape[0], 4), dtype=np.float64)
    homogeneous_points[:, :3] = points[:, :3]
    rectified_camera_points = (
        calibration.r_rect_00
        @ calibration.tr_velo_to_cam
        @ homogeneous_points.T
    )
    depths = rectified_camera_points[2]
    projected_points = calibration.p_rect_02 @ rectified_camera_points
    with np.errstate(divide='ignore', invalid='ignore'):
        pixel_x = projected_points[0] / projected_points[2]
        pixel_y = projected_points[1] / projected_points[2]

    valid = (
        np.isfinite(pixel_x)
        & np.isfinite(pixel_y)
        & np.isfinite(depths)
        & np.isfinite(points[:, 3])
        & (depths >= min_depth)
        & (pixel_x >= 0.0)
        & (pixel_x < image_width)
        & (pixel_y >= 0.0)
        & (pixel_y < image_height)
    )
    if max_depth is not None:
        valid &= depths <= max_depth

    return ProjectedPointCloud(
        pixels=np.column_stack((pixel_x[valid], pixel_y[valid])),
        depths=depths[valid],
        intensities=points[valid, 3],
        camera_points=rectified_camera_points[:3, valid].T,
        velodyne_points=points[valid, :3],
    )


def project_velodyne_to_image(
    points,
    calibration,
    image_width,
    image_height,
    min_depth=0.1,
    max_depth=None,
):
    projected = project_velodyne_to_image_details(
        points,
        calibration,
        image_width,
        image_height,
        min_depth=min_depth,
        max_depth=max_depth,
    )
    return projected.pixels, projected.depths, projected.intensities


def draw_lidar_overlay(
    image,
    pixels,
    depths,
    max_depth=80.0,
    point_radius=2,
):
    if image.ndim != 3 or image.shape[2] != 3:
        raise RuntimeError(
            f'Expected BGR image shape (H, W, 3), got {image.shape}'
        )
    if len(pixels) != len(depths):
        raise RuntimeError(
            'Projected pixel and depth counts do not match: '
            f'pixels={len(pixels)}, depths={len(depths)}'
        )
    if max_depth <= 0.0:
        raise RuntimeError(f'max_depth must be positive: {max_depth}')
    if point_radius < 0:
        raise RuntimeError(
            f'point_radius must not be negative: {point_radius}'
        )

    overlay = image.copy()
    if len(pixels) == 0:
        return overlay
    height, width = overlay.shape[:2]
    pixel_coordinates = np.rint(pixels).astype(np.int32)
    pixel_coordinates[:, 0] = np.clip(pixel_coordinates[:, 0], 0, width - 1)
    pixel_coordinates[:, 1] = np.clip(pixel_coordinates[:, 1], 0, height - 1)
    normalized_depth = np.clip(depths / max_depth, 0.0, 1.0)
    color_values = np.rint((1.0 - normalized_depth) * 255.0).astype(np.uint8)
    colors = cv2.applyColorMap(
        color_values.reshape(-1, 1),
        cv2.COLORMAP_TURBO,
    ).reshape(-1, 3)
    draw_order = np.argsort(depths)[::-1]
    pixel_coordinates = pixel_coordinates[draw_order]
    colors = colors[draw_order]

    for offset_y in range(-point_radius, point_radius + 1):
        for offset_x in range(-point_radius, point_radius + 1):
            if offset_x ** 2 + offset_y ** 2 > point_radius ** 2:
                continue
            draw_x = pixel_coordinates[:, 0] + offset_x
            draw_y = pixel_coordinates[:, 1] + offset_y
            valid = (
                (draw_x >= 0)
                & (draw_x < width)
                & (draw_y >= 0)
                & (draw_y < height)
            )
            overlay[draw_y[valid], draw_x[valid]] = colors[valid]
    return overlay
