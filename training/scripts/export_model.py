#!/usr/bin/env python3
"""Export an Ultralytics checkpoint for runtime inference."""

import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('--format', default='onnx')
    args = parser.parse_args()
    subprocess.run(
        ['yolo', 'export', f'model={args.model}', f'format={args.format}'],
        check=True,
    )


if __name__ == '__main__':
    main()
