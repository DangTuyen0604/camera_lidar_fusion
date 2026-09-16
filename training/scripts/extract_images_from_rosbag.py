#!/usr/bin/env python3
"""Extract a ROS Image or CompressedImage topic into deterministic files."""

import argparse
import csv
from pathlib import Path


def _load_ros_dependencies():
    try:
        from cv_bridge import CvBridge
        from rclpy.serialization import deserialize_message
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            'ROS 2 Python, cv_bridge and rosbag2_py are required'
        ) from error
    return CvBridge, deserialize_message, rosbag2_py, get_message


def _open_reader(bag_path, rosbag2_py):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)
    return reader


def extract_images(bag, output, topic, image_format='png', limit=0):
    import cv2

    if limit < 0:
        raise RuntimeError('limit must not be negative')
    if image_format not in ('png', 'jpg'):
        raise RuntimeError('image_format must be png or jpg')

    bag = Path(bag).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    if not bag.exists():
        raise RuntimeError(f'Rosbag does not exist: {bag}')
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f'Output directory must be empty: {output}')
    output.mkdir(parents=True, exist_ok=True)

    CvBridge, deserialize_message, rosbag2_py, get_message = (
        _load_ros_dependencies()
    )
    reader = _open_reader(bag, rosbag2_py)
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    message_type_name = topic_types.get(topic)
    supported_types = {
        'sensor_msgs/msg/Image',
        'sensor_msgs/msg/CompressedImage',
    }
    if message_type_name not in supported_types:
        raise RuntimeError(
            f'Topic {topic} must be Image or CompressedImage; '
            f'available type={message_type_name}'
        )

    message_type = get_message(message_type_name)
    bridge = CvBridge()
    manifest_path = output / 'manifest.csv'
    count = 0
    with manifest_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['index', 'timestamp_ns', 'topic', 'file'])
        while reader.has_next() and (limit == 0 or count < limit):
            current_topic, serialized, recorded_timestamp = reader.read_next()
            if current_topic != topic:
                continue
            message = deserialize_message(serialized, message_type)
            if message_type_name == 'sensor_msgs/msg/Image':
                image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            else:
                image = bridge.compressed_imgmsg_to_cv2(
                    message,
                    desired_encoding='bgr8',
                )
            filename = f'{count:010d}_{recorded_timestamp}.{image_format}'
            image_path = output / filename
            if not cv2.imwrite(str(image_path), image):
                raise RuntimeError(f'Cannot write extracted image: {image_path}')
            writer.writerow([count, recorded_timestamp, topic, filename])
            count += 1

    if count == 0:
        manifest_path.unlink(missing_ok=True)
        raise RuntimeError(f'No messages found on image topic: {topic}')
    return count, manifest_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--topic', default='/kitti/camera/image_raw')
    parser.add_argument('--format', choices=('png', 'jpg'), default='png')
    parser.add_argument('--limit', type=int, default=0)
    args = parser.parse_args()
    try:
        count, manifest = extract_images(
            args.bag,
            args.output,
            args.topic,
            image_format=args.format,
            limit=args.limit,
        )
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    print(f'Extracted {count} images; manifest: {manifest}')


if __name__ == '__main__':
    main()
