#!/usr/bin/env python3
"""Export an Ultralytics checkpoint for runtime inference."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('--format', default='onnx')
    parser.add_argument('--output-metadata', type=Path)
    args = parser.parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit('Ultralytics is not installed') from error
    exported_path = Path(YOLO(args.model).export(format=args.format)).resolve()
    metadata = {
        'source_checkpoint': str(Path(args.model).resolve()),
        'format': args.format,
        'exported_model': str(exported_path),
        'size_bytes': exported_path.stat().st_size,
        'reproduce': (
            f'python3 training/scripts/export_model.py {args.model} '
            f'--format {args.format}'
        ),
    }
    if args.format == 'onnx':
        try:
            import onnxruntime
            session = onnxruntime.InferenceSession(
                str(exported_path),
                providers=['CPUExecutionProvider'],
            )
            model_metadata = session.get_modelmeta().custom_metadata_map
            metadata['input_shape'] = session.get_inputs()[0].shape
            metadata['output_shape'] = session.get_outputs()[0].shape
            metadata['class_names'] = model_metadata.get('names', '')
        except ImportError as error:
            raise SystemExit(
                'onnxruntime is required to verify ONNX export'
            ) from error
    metadata_path = args.output_metadata or exported_path.with_suffix(
        '.metadata.json'
    )
    metadata_path.write_text(
        json.dumps(metadata, indent=2) + '\n',
        encoding='utf-8',
    )
    print(f'Exported model: {exported_path}')
    print(f'Export metadata: {metadata_path}')


if __name__ == '__main__':
    main()
