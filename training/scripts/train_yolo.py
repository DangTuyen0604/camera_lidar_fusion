#!/usr/bin/env python3
"""Start Ultralytics training from a YAML configuration."""

import argparse
import subprocess

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config')
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    command = ['yolo', 'detect', 'train']
    command.extend(f'{key}={value}' for key, value in config.items())
    subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
