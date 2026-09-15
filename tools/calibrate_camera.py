#!/usr/bin/env python3
"""Calibrate a pinhole camera from chessboard images and write ROS YAML."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--columns', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--square-size', type=float, default=0.025)
    args = parser.parse_args()
    if min(args.columns, args.rows) < 2 or args.square_size <= 0:
        parser.error('Invalid chessboard dimensions')

    pattern = (args.columns, args.rows)
    object_template = np.zeros((args.columns * args.rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[
        0:args.columns, 0:args.rows
    ].T.reshape(-1, 2) * args.square_size
    object_points = []
    image_points = []
    image_size = None
    for path in sorted(args.images.glob('*')):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        found, corners = cv2.findChessboardCorners(image, pattern)
        if not found:
            continue
        refined = cv2.cornerSubPix(
            image, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        object_points.append(object_template)
        image_points.append(refined)
    if len(object_points) < 5 or image_size is None:
        raise SystemExit('At least five readable chessboard views are required')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    document = {
        'image_width': image_size[0],
        'image_height': image_size[1],
        'camera_name': 'calibrated_camera',
        'distortion_model': 'plumb_bob',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.reshape(-1).tolist()},
        'distortion_coefficients': {
            'rows': 1, 'cols': int(distortion.size),
            'data': distortion.reshape(-1).tolist(),
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3, 'data': np.eye(3).reshape(-1).tolist(),
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': np.column_stack((matrix, np.zeros(3))).reshape(-1).tolist(),
        },
        'reprojection_rms_px': float(rms),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
    print(f'Used {len(object_points)} views; RMS={rms:.3f} px')


if __name__ == '__main__':
    main()
