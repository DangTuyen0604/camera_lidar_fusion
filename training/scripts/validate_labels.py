#!/usr/bin/env python3
"""Validate normalized YOLO label files."""

import argparse
from pathlib import Path


def validate_file(path, number_of_classes=None):
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
            continue
        if number_of_classes is not None and class_id >= number_of_classes:
            errors.append(
                f'{path}:{line_number}: class {class_id} is outside '
                f'[0, {number_of_classes - 1}]'
            )
        _, _, width, height = coordinates
        if width <= 0.0 or height <= 0.0:
            errors.append(f'{path}:{line_number}: box width/height must be positive')
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('labels', type=Path)
    parser.add_argument('--images', type=Path)
    parser.add_argument('--num-classes', type=int)
    args = parser.parse_args()
    if args.num_classes is not None and args.num_classes <= 0:
        parser.error('--num-classes must be positive')
    if not args.labels.is_dir():
        raise SystemExit(f'Label directory not found: {args.labels}')
    errors = []
    label_paths = sorted(args.labels.rglob('*.txt'))
    if not label_paths:
        raise SystemExit(f'No YOLO label files found under {args.labels}')
    for path in label_paths:
        errors.extend(validate_file(path, args.num_classes))
    if args.images is not None:
        if not args.images.is_dir():
            errors.append(f'Image directory not found: {args.images}')
        else:
            image_stems = {
                path.stem for path in args.images.rglob('*')
                if path.suffix.lower() in {'.jpg', '.jpeg', '.png'}
            }
            label_stems = {path.stem for path in label_paths}
            missing_labels = sorted(image_stems - label_stems)
            orphan_labels = sorted(label_stems - image_stems)
            if missing_labels:
                errors.append(
                    f'Images without labels: {", ".join(missing_labels[:10])}'
                )
            if orphan_labels:
                errors.append(
                    f'Labels without images: {", ".join(orphan_labels[:10])}'
                )
    if errors:
        raise SystemExit('\n'.join(errors))
    print(f'Validated labels under {args.labels}')


if __name__ == '__main__':
    main()
