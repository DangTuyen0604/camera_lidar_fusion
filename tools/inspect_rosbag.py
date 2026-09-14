#!/usr/bin/env python3
"""Print rosbag metadata through the installed ROS 2 CLI."""

import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Path to a rosbag2 directory')
    args = parser.parse_args()
    subprocess.run(['ros2', 'bag', 'info', args.bag], check=True)


if __name__ == '__main__':
    main()
