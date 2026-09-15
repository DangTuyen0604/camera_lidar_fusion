#!/usr/bin/env python3
"""Create latency plots and a per-scenario summary from benchmark CSV."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input', default='benchmarks/results/benchmark_summary.csv'
    )
    parser.add_argument('--output', default='benchmarks/results/latency_plot.png')
    parser.add_argument(
        '--summary', default='benchmarks/results/calibration_results.csv'
    )
    args = parser.parse_args()

    data = pd.read_csv(args.input)
    if data.empty:
        raise SystemExit('Benchmark CSV contains no samples')
    successful = data[data['return_code'] == 0]
    if successful.empty:
        raise SystemExit('Benchmark CSV contains no successful runs')
    summary = successful.groupby('scenario')['latency_ms'].agg(
        samples='count', mean_ms='mean', median_ms='median',
        p95_ms=lambda values: values.quantile(0.95), maximum_ms='max'
    )
    summary.to_csv(args.summary)

    figure, axis = plt.subplots(figsize=(9, 5))
    for scenario, group in successful.groupby('scenario'):
        axis.plot(group['run'], group['latency_ms'], marker='o', label=scenario)
    axis.set(title='Camera–LiDAR benchmark latency', xlabel='Run', ylabel='Latency (ms)')
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=150)
    print(summary.to_string())


if __name__ == '__main__':
    main()
