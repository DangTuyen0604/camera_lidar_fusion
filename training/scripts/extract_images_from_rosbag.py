#!/usr/bin/env python3
"""Document the reproducible image extraction entry point."""

import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--topic', default='/kitti/camera/image_raw')
    parser.parse_args()
    raise SystemExit(
        'Image extraction needs a rosbag2_py storage backend matching the bag. '
        'Install that backend before implementing this dataset-specific step.'
    )


if __name__ == '__main__':
    main()
