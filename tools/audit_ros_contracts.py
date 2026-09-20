#!/usr/bin/env python3
"""Audit canonical TF ownership and cross-package topic/frame contracts."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FRAMES = {
    'base_footprint', 'base_link', 'lidar_link', 'camera_link',
    'camera_optical_frame'}
REQUIRED_TOPICS = {
    '/cmd_vel', '/odom', '/scan', '/camera/image_raw',
    '/fusion/detections_3d', '/navigation/detection_obstacles',
    '/mission/state', '/mission/cargo'}


def main():
    model = ROOT / 'src/openamrobot_description/urdf/mobile_robot.urdf.xacro'
    expanded = subprocess.check_output(['xacro', str(model)], text=True)
    robot = ET.fromstring(expanded)
    links = [element.attrib['name'] for element in robot.findall('link')]
    if len(links) != len(set(links)):
        raise SystemExit('duplicate link definitions in canonical robot')
    missing = REQUIRED_FRAMES - set(links)
    if missing:
        raise SystemExit(f'missing canonical frames: {sorted(missing)}')
    children = [joint.find('child').attrib['link'] for joint in robot.findall('joint')]
    duplicate_children = {child for child in children if children.count(child) > 1}
    if duplicate_children:
        raise SystemExit(f'duplicate TF parents for: {sorted(duplicate_children)}')
    roots = set(links) - set(children)
    if roots != {'base_footprint'}:
        raise SystemExit(f'expected only base_footprint as TF root, found {sorted(roots)}')

    source = '\n'.join(
        path.read_text(encoding='utf-8', errors='ignore')
        for path in (ROOT / 'src').rglob('*') if path.is_file())
    missing_topics = REQUIRED_TOPICS - {topic for topic in REQUIRED_TOPICS if topic in source}
    if missing_topics:
        raise SystemExit(f'missing topic contracts: {sorted(missing_topics)}')
    print(f'ROS contracts OK: {len(links)} unique links, one TF root, '
          f'{len(REQUIRED_TOPICS)} required topics')


if __name__ == '__main__':
    main()
