#!/usr/bin/env python3
"""
Rasterize the warehouse SDF collision geometry into the Nav2 static map.

Every static collision box/cylinder/sphere that rises above the floor is
projected onto the ground plane.  The map must be fine enough for AMCL: its
likelihood field is constant inside a cell, so at 0.2 m/cell a 0.1-0.2 m pose
error is unobservable, which is larger than the 8 cm docking contract.

Usage:
    python3 tools/generate_warehouse_map.py            # 0.05 m/cell
    python3 tools/generate_warehouse_map.py --resolution 0.1
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'src/warehouse_simulation/worlds/warehouse.sdf'
MODELS = ROOT / 'src/warehouse_simulation/models'
MAPS = ROOT / 'src/navigation_bringup/maps'
ORIGIN = (-9.0, -7.0)
SIZE = (18.0, 14.0)
FLOOR_TOP = 0.05       # warehouse.sdf ground plane height
MIN_HEIGHT = 0.03      # ignore geometry within 3 cm of the floor
# Overlap below 5 mm is authoring round-off (yaw 3.14 is not pi), not contact.
TOLERANCE = 0.005
OCCUPIED, FREE = 0, 9  # P2 values with maxval 9, as map_server expects


def pose_matrix(text):
    """4x4 transform of an SDF 'x y z roll pitch yaw' pose."""
    values = [float(v) for v in (text or '').split()] + [0.0] * 6
    x, y, z, roll, pitch, yaw = values[:6]
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                      @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                      @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    matrix[:3, 3] = (x, y, z)
    return matrix


def pose_of(element):
    node = element.find('pose')
    return pose_matrix(node.text if node is not None else '')


def half_extents(geometry):
    """Local half extents of a collision primitive, or None to skip it."""
    box = geometry.find('box')
    if box is not None:
        return np.array([float(v) for v in box.find('size').text.split()]) / 2
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.find('radius').text)
        return np.array([radius, radius, float(cylinder.find('length').text) / 2])
    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.find('radius').text)
        return np.array([radius, radius, radius])
    return None


def footprints(model, frame):
    """Yield (ground polygon, is_round) for each collision primitive of a model."""
    for link in model.findall('link'):
        link_frame = frame @ pose_of(link)
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            extents = half_extents(geometry) if geometry is not None else None
            if extents is None:
                continue
            transform = link_frame @ pose_of(collision)
            corners = np.array([[sx, sy, sz, 1.0] for sx in (-1, 1)
                                for sy in (-1, 1) for sz in (-1, 1)])
            corners[:, :3] *= extents
            world = (transform @ corners.T).T[:, :3]
            if world[:, 2].max() < FLOOR_TOP + MIN_HEIGHT:
                continue
            round_ = geometry.find('box') is None
            yield world[:, :2], round_, transform, extents


def _counter_clockwise(polygon):
    x, y = polygon[:, 0], polygon[:, 1]
    return np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0


def rasterize(resolution):
    width, height = (round(s / resolution) for s in SIZE)
    grid = np.full((height, width), FREE, np.uint8)
    world = ET.parse(WORLD).getroot().find('world')
    models = [(m, pose_of(m)) for m in world.findall('model')]
    for include in world.findall('include'):
        name = include.find('uri').text.replace('model://', '').strip()
        model = ET.parse(MODELS / name / 'model.sdf').getroot().find('model')
        models.append((model, pose_of(include)))

    # World coordinates of every cell centre (row 0 is the top of the image).
    xs = ORIGIN[0] + (np.arange(width) + 0.5) * resolution
    ys = ORIGIN[1] + (height - np.arange(height) - 0.5) * resolution
    cx, cy = np.meshgrid(xs, ys)

    half = resolution / 2
    for model, frame in models:
        for polygon, round_, transform, extents in footprints(model, frame):
            # A cell is occupied when the obstacle overlaps its interior (the
            # conservative rule of the original map); merely touching an edge
            # does not count.
            if round_ and abs(transform[2, 2]) > 0.99:  # upright cylinder/sphere
                ox, oy = transform[:2, 3]
                dx = np.maximum(np.abs(cx - ox) - half, 0.0)
                dy = np.maximum(np.abs(cy - oy) - half, 0.0)
                grid[np.hypot(dx, dy) < extents[0] - TOLERANCE] = OCCUPIED
                continue
            # Minkowski sum with the cell square, then a strict inside test.
            grown = np.concatenate([polygon + (sx * half, sy * half)
                                    for sx in (-1, 1) for sy in (-1, 1)])
            hull = cv2.convexHull(grown.astype(np.float32)).reshape(-1, 2)
            sign = 1.0 if _counter_clockwise(hull) else -1.0
            inside = np.ones_like(cx, dtype=bool)
            for (x1, y1), (x2, y2) in zip(hull, np.roll(hull, -1, axis=0)):
                cross = (x2 - x1) * (cy - y1) - (y2 - y1) * (cx - x1)
                inside &= sign * cross > TOLERANCE * np.hypot(x2 - x1, y2 - y1)
            grid[inside] = OCCUPIED
    return grid


def write(grid, resolution):
    height, width = grid.shape
    lines = ['P2',
             f'# Warehouse static map, {resolution} m/cell, generated from '
             'warehouse.sdf collision geometry by tools/generate_warehouse_map.py',
             f'{width} {height}', '9']
    lines += [' '.join(str(v) for v in row) for row in grid]
    (MAPS / 'warehouse_map.pgm').write_text('\n'.join(lines) + '\n')
    (MAPS / 'warehouse_map.yaml').write_text(
        'image: warehouse_map.pgm\nmode: trinary\n'
        f'resolution: {resolution}\n'
        f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--resolution', type=float, default=0.05)
    args = parser.parse_args()
    grid = rasterize(args.resolution)
    write(grid, args.resolution)
    print(f'{grid.shape[1]}x{grid.shape[0]} cells at {args.resolution} m, '
          f'{int((grid == OCCUPIED).sum())} occupied')


if __name__ == '__main__':
    main()
