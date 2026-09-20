#!/usr/bin/env python3
"""Create all Gate-6.2 plots strictly from recorded benchmark CSV."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input', default='benchmark_results/raw_results.csv'
    )
    parser.add_argument('--output-dir', default='benchmark_results')
    args = parser.parse_args()

    data = pd.read_csv(args.input)
    if data.empty:
        raise SystemExit('Benchmark CSV contains no samples')
    successful = data[data['stack_exit_code'] == 0]
    if successful.empty:
        raise SystemExit('Benchmark CSV contains no successful runs')
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    def bars(filename, columns, title, ylabel):
        grouped = successful.groupby('scenario')[columns].mean()
        axis = grouped.plot.bar(figsize=(13, 6))
        axis.set(title=title, xlabel='Scenario', ylabel=ylabel)
        axis.grid(True, axis='y', alpha=0.3)
        axis.figure.tight_layout()
        axis.figure.savefig(output / filename, dpi=150)
        plt.close(axis.figure)

    bars('latency.png', ['latency_mean_ms', 'latency_p95_ms', 'latency_p99_ms'],
         'Fusion latency', 'Milliseconds')
    bars('fusion_success_rate.png', ['fusion_success_rate'],
         'Fusion success rate', 'Ratio')
    bars('calibration_quality.png', ['calibration_quality', 'projection_error_px'],
         'Calibration quality', 'Score / pixels')
    bars('mission_duration.png', ['mission_duration_s'],
         'Mission duration', 'Seconds')
    bars('resource_usage.png', ['cpu_mean_percent', 'memory_peak_mb'],
         'Resource usage', 'Percent / MiB')
    delay = successful[successful.scenario.str.contains('delay|baseline')]
    noise = successful[successful.scenario.str.contains('noise|density|baseline')]
    for filename, frame, title in (
            ('delay_sensitivity.png', delay, 'Timestamp-delay sensitivity'),
            ('noise_sensitivity.png', noise, 'Noise and density sensitivity')):
        values = frame.groupby('scenario')[
            ['latency_p95_ms', 'fusion_success_rate']].mean()
        axis = values.plot.bar(figsize=(10, 5))
        axis.set_title(title)
        axis.grid(True, axis='y', alpha=0.3)
        axis.figure.tight_layout()
        axis.figure.savefig(output / filename, dpi=150)
        plt.close(axis.figure)

    violations = []
    if (successful.collision_count != 0).any():
        violations.append('collision_count != 0')
    if (successful.person_stop_success != 1).any():
        violations.append('person_stop_success != 100%')
    if successful.mission_success.mean() < 0.90:
        violations.append('mission_success_rate < 90%')
    if (successful.latency_p95_ms >= 150).any():
        violations.append('p95_fusion_latency >= 150 ms')
    if (successful.docking_error_m >= 0.08).any():
        violations.append('docking_error >= 0.08 m')
    if violations:
        raise SystemExit('Benchmark gates failed: ' + '; '.join(violations))
    print(f'Created seven plots from {len(successful)} real runs')


if __name__ == '__main__':
    main()
