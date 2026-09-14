#!/usr/bin/env python3
"""Validate normalized YOLO label files."""

import argparse
from pathlib import Path


def validate_file(path):
    errors = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        fields = line.split()
        if len(fields) != 5:
            errors.append(f'{path}:{line_number}: expected 5 fields')
            continue
        try:
            class_id = int(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError:
            errors.append(f'{path}:{line_number}: non-numeric field')
            continue
        if class_id < 0 or any(value < 0.0 or value > 1.0 for value in coordinates):
            errors.append(f'{path}:{line_number}: value outside YOLO range')
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('labels', type=Path)
    args = parser.parse_args()
    errors = []
    for path in sorted(args.labels.rglob('*.txt')):
        errors.extend(validate_file(path))
    if errors:
        raise SystemExit('\n'.join(errors))
    print(f'Validated labels under {args.labels}')


if __name__ == '__main__':
    main()
