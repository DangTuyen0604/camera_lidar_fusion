#!/usr/bin/env python3
"""Run every Day-6 scenario against a live ROS graph and write real results."""

import argparse
from pathlib import Path

from csv_writer import RAW_FIELDS, write_rows, write_summary
from scenario_runner import load_scenarios, run_scenario


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='benchmarks/scenarios.yaml')
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--startup-timeout', type=float, default=45.0)
    parser.add_argument('--duration', type=float, default=90.0)
    parser.add_argument('--output-dir', default='benchmark_results')
    parser.add_argument('--scenario', action='append', dest='selected')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.runs < 5 and not args.dry_run:
        parser.error('Gate 6.2 requires at least five live runs per configuration')
    scenarios = load_scenarios(args.config)
    if args.selected:
        scenarios = [item for item in scenarios if item.name in args.selected]
        unknown = set(args.selected) - {item.name for item in scenarios}
        if unknown:
            parser.error(f'unknown scenarios: {sorted(unknown)}')
    if args.dry_run:
        for item in scenarios:
            print(item.name, item.parameters)
        return
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario in scenarios:
        count = max(args.runs, scenario.runs)
        for run_number in range(1, count + 1):
            print(f'[{scenario.name}] live run {run_number}/{count}', flush=True)
            row = run_scenario(
                scenario, run_number, output, args.startup_timeout, args.duration)
            missing = [name for name in RAW_FIELDS if name not in row]
            if missing:
                raise RuntimeError(
                    f'{scenario.name} did not produce real metrics: {missing}')
            rows.append(row)
            write_rows(output / 'raw_results.csv', rows)
    write_summary(output / 'summary.csv', rows)
    print(f'Wrote {len(rows)} live runs to {output}')


if __name__ == '__main__':
    main()
