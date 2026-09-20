"""Stable raw and aggregate benchmark CSV schemas."""

import csv
from pathlib import Path
import statistics


RAW_FIELDS = (
    'scenario', 'run', 'latency_mean_ms', 'latency_p95_ms', 'latency_p99_ms',
    'projection_error_px', 'xyz_error_m', 'fusion_success_rate',
    'calibration_quality', 'replan_count', 'stop_events', 'collision_count',
    'mission_duration_s', 'mission_success', 'person_stop_success',
    'docking_error_m', 'ghost_obstacle_lifetime_s', 'cpu_mean_percent',
    'cpu_peak_percent', 'memory_peak_mb', 'wall_time_s', 'stack_exit_code')


def write_rows(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=RAW_FIELDS, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def _mean(rows, field):
    return statistics.fmean(float(row[field]) for row in rows)


def write_summary(path, rows):
    groups = {}
    for row in rows:
        groups.setdefault(row['scenario'], []).append(row)
    fields = ['scenario', 'runs'] + [
        name for name in RAW_FIELDS if name not in ('scenario', 'run')]
    summaries = []
    for scenario, group in groups.items():
        summary = {'scenario': scenario, 'runs': len(group)}
        for field in fields[2:]:
            summary[field] = _mean(group, field)
        summaries.append(summary)
    with Path(path).open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
