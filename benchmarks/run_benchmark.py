#!/usr/bin/env python3
"""Repeat a command and write timing measurements as CSV."""

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import time

import yaml


def run_once(command, timeout):
    started = time.perf_counter()
    result = subprocess.run(command, check=False, timeout=timeout)
    return (time.perf_counter() - started) * 1000.0, result.returncode


def scenarios_from_config(path):
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    return document.get('scenarios', [])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='manual')
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument('--config')
    parser.add_argument(
        '--output', default='benchmarks/results/benchmark_summary.csv'
    )
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    scenarios = scenarios_from_config(args.config) if args.config else [{
        'name': args.scenario,
        'runs': args.runs,
        'timeout_sec': args.timeout,
        'command': args.command[1:] if args.command[:1] == ['--'] else args.command,
    }]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for scenario in scenarios:
        command = scenario.get('command', [])
        if not command:
            raise SystemExit(f"Scenario {scenario.get('name')} has no command")
        for run_number in range(1, int(scenario.get('runs', 1)) + 1):
            latency_ms, return_code = run_once(
                command, float(scenario.get('timeout_sec', args.timeout))
            )
            rows.append({
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'scenario': scenario.get('name', 'unnamed'),
                'run': run_number,
                'latency_ms': f'{latency_ms:.3f}',
                'return_code': return_code,
                'command': ' '.join(command),
            })
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {len(rows)} samples to {output}')
    raise SystemExit(max(row['return_code'] for row in rows))


if __name__ == '__main__':
    main()
