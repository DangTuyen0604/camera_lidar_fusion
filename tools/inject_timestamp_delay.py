#!/usr/bin/env python3
"""Shift KITTI timestamps by a fixed delay while preserving formatting."""

import argparse
from datetime import datetime, timedelta
from pathlib import Path


FORMAT = '%Y-%m-%d %H:%M:%S.%f'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--delay-ms', required=True, type=float)
    args = parser.parse_args()
    shifted = []
    for line_number, line in enumerate(
        args.input.read_text(encoding='utf-8').splitlines(), start=1
    ):
        try:
            timestamp = datetime.strptime(line.strip(), FORMAT)
        except ValueError as error:
            raise SystemExit(f'Invalid timestamp on line {line_number}: {line}') from error
        shifted.append(
            (timestamp + timedelta(milliseconds=args.delay_ms)).strftime(FORMAT)
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(shifted) + '\n', encoding='utf-8')
    print(f'Shifted {len(shifted)} timestamps by {args.delay_ms} ms')


if __name__ == '__main__':
    main()
