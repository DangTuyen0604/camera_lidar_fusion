#!/usr/bin/env python3
"""Evaluate an Ultralytics model on a YOLO dataset."""

import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('dataset')
    args = parser.parse_args()
    subprocess.run(
        ['yolo', 'detect', 'val', f'model={args.model}', f'data={args.dataset}'],
        check=True,
    )


if __name__ == '__main__':
    main()
