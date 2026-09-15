#!/usr/bin/env python3
"""Generate extrinsic variants described by an experiment YAML file."""

import argparse
import csv
from pathlib import Path

from perturb_extrinsic import perturb_document
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('reference', type=Path)
    parser.add_argument(
        '--scenarios', type=Path,
        default=Path('experiments/calibration_scenarios.yaml'),
    )
    parser.add_argument('--output-dir', type=Path, default=Path('runs/calibration'))
    args = parser.parse_args()
    with args.reference.open(encoding='utf-8') as stream:
        reference = yaml.safe_load(stream)
    with args.scenarios.open(encoding='utf-8') as stream:
        scenarios = (yaml.safe_load(stream) or {}).get('scenarios', [])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario in scenarios:
        name = scenario['name']
        translation = scenario.get('translation_m', [0, 0, 0])
        rotation = scenario.get('rotation_deg', [0, 0, 0])
        output = args.output_dir / f'{name}.yaml'
        with output.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                perturb_document(reference, translation, rotation),
                stream, sort_keys=False,
            )
        rows.append({
            'scenario': name,
            'translation_drift_m': sum(value * value for value in translation) ** 0.5,
            'rotation_input_deg': sum(value * value for value in rotation) ** 0.5,
            'extrinsic_path': output,
        })
    report = args.output_dir / 'scenario_manifest.csv'
    with report.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f'Generated {len(rows)} scenarios and {report}')


if __name__ == '__main__':
    main()
