#!/usr/bin/env python3
"""Inspect and optionally validate rosbag metadata."""

import argparse
from pathlib import Path
import subprocess


def read_metadata(bag):
    try:
        import rosbag2_py
    except ImportError as error:
        raise RuntimeError('rosbag2_py is required to validate a bag') from error
    bag = Path(bag).expanduser().resolve()
    if not bag.exists():
        raise RuntimeError(f'Rosbag does not exist: {bag}')
    return rosbag2_py.Info().read_metadata(str(bag), '')


def validate_metadata(metadata, required_topics, minimum_messages):
    topic_counts = {
        entry.topic_metadata.name: entry.message_count
        for entry in metadata.topics_with_message_count
    }
    errors = []
    for topic in required_topics:
        count = topic_counts.get(topic, 0)
        if count < minimum_messages:
            errors.append(
                f'{topic}: expected at least {minimum_messages} messages, '
                f'found {count}'
            )
    if metadata.message_count <= 0:
        errors.append('bag contains no messages')
    if errors:
        raise RuntimeError('Rosbag validation failed:\n' + '\n'.join(errors))
    return topic_counts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Path to a rosbag2 directory')
    parser.add_argument(
        '--require-topic',
        action='append',
        default=[],
        help='topic that must exist; repeat for multiple topics',
    )
    parser.add_argument('--minimum-messages', type=int, default=1)
    args = parser.parse_args()
    if args.minimum_messages < 1:
        parser.error('--minimum-messages must be positive')
    subprocess.run(['ros2', 'bag', 'info', args.bag], check=True)
    if args.require_topic:
        try:
            metadata = read_metadata(args.bag)
            counts = validate_metadata(
                metadata,
                args.require_topic,
                args.minimum_messages,
            )
        except RuntimeError as error:
            raise SystemExit(str(error)) from error
        print('Validated topics:')
        for topic in args.require_topic:
            print(f'  {topic}: {counts[topic]} messages')


if __name__ == '__main__':
    main()
