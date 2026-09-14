from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    bbox: tuple

    def to_dict(self):
        return {
            'class_id': self.class_id,
            'label': self.label,
            'confidence': round(self.confidence, 6),
            'bbox': {
                'x1': self.bbox[0],
                'y1': self.bbox[1],
                'x2': self.bbox[2],
                'y2': self.bbox[3],
            },
        }


def detection_from_dict(data):
    bbox = data['bbox']

    return Detection(
        class_id=int(data['class_id']),
        label=str(data['label']),
        confidence=float(data['confidence']),
        bbox=(
            int(bbox['x1']),
            int(bbox['y1']),
            int(bbox['x2']),
            int(bbox['y2']),
        ),
    )


@dataclass(frozen=True)
class FusedDetection:
    detection: object
    distance_m: float
    position_camera: tuple
    point_count: int
    raw_point_count: int
    selected_pixels: np.ndarray = field(repr=False)

    def to_dict(self):
        result = self.detection.to_dict()
        result.update({
            'distance_m': round(self.distance_m, 3),
            'position_3d': {
                'frame_id': 'camera_optical_frame',
                'x': round(self.position_camera[0], 3),
                'y': round(self.position_camera[1], 3),
                'z': round(self.position_camera[2], 3),
            },
            'lidar_points': self.point_count,
            'lidar_points_before_filter': self.raw_point_count,
        })
        return result


def fuse_detections(
    detections,
    projected,
    minimum_points=3,
    bbox_shrink=0.08,
    mad_scale=3.0,
    minimum_depth_window=0.75,
):
    fused_detections = []
    for detection in detections:
        x1, y1, x2, y2 = detection.bbox
        margin_x = (x2 - x1) * bbox_shrink
        margin_y = (y2 - y1) * bbox_shrink
        pixels = projected.pixels
        inside = (
            (pixels[:, 0] >= x1 + margin_x)
            & (pixels[:, 0] <= x2 - margin_x)
            & (pixels[:, 1] >= y1 + margin_y)
            & (pixels[:, 1] <= y2 - margin_y)
        )
        raw_indices = np.flatnonzero(inside)
        if len(raw_indices) < minimum_points:
            continue

        raw_depths = projected.depths[raw_indices]
        median_depth = np.median(raw_depths)
        absolute_deviation = np.abs(raw_depths - median_depth)
        mad = np.median(absolute_deviation)
        depth_window = max(minimum_depth_window, mad_scale * 1.4826 * mad)
        selected_indices = raw_indices[absolute_deviation <= depth_window]
        if len(selected_indices) < minimum_points:
            continue

        position = np.median(
            projected.camera_points[selected_indices],
            axis=0,
        )
        fused_detections.append(FusedDetection(
            detection=detection,
            distance_m=float(np.median(projected.depths[selected_indices])),
            position_camera=tuple(float(value) for value in position),
            point_count=len(selected_indices),
            raw_point_count=len(raw_indices),
            selected_pixels=projected.pixels[selected_indices],
        ))
    return fused_detections
