#!/usr/bin/env python3
"""Evaluate an Ultralytics model on a YOLO dataset."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('dataset')
    parser.add_argument('--output', type=Path, default=Path('evaluation.json'))
    parser.add_argument('--imgsz', type=int, default=640)
    args = parser.parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit('Ultralytics is not installed') from error
    metrics = YOLO(args.model).val(data=args.dataset, imgsz=args.imgsz)
    results = {
        'model': str(Path(args.model).resolve()),
        'dataset': str(Path(args.dataset).resolve()),
        'precision': float(metrics.box.mp),
        'recall': float(metrics.box.mr),
        'map50': float(metrics.box.map50),
        'map50_95': float(metrics.box.map),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(results, indent=2) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
