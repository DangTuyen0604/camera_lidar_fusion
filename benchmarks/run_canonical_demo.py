#!/usr/bin/env python3
"""Repeat the one canonical end-to-end demo and persist measured results."""

import argparse
import csv
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import tempfile


METRICS = (
    'camera_rate_hz', 'pointcloud_rate_hz', 'detection_rate_hz',
    'fusion_rate_hz', 'sync_delay_mean_ms', 'goal_duration_s',
    'obstacle_reaction_latency_ms', 'pipeline_end_to_end_mean_ms',
    'obstacle_expiry_s', 'minimum_robot_obstacle_clearance_m',
)


def terminate(process):
    """Stop the launch process group and require clean teardown."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10.0)


def run_once(run_number, seed, domain_id, probe, log_path):
    """Launch a fresh stack and execute the same canonical scenario once."""
    environment = os.environ.copy()
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    environment['ROS_DOMAIN_ID'] = str(domain_id)
    with log_path.open('w', encoding='utf-8') as log:
        stack = subprocess.Popen(
            ['ros2', 'launch', 'fusion_bringup',
             'final_demo.launch.py', 'use_rviz:=false'],
            env=environment, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            with tempfile.TemporaryDirectory(prefix='canonical-demo-') as temporary:
                output = Path(temporary) / 'result.json'
                command = [
                    sys.executable, str(probe), '--run', str(run_number),
                    '--seed', str(seed), '--output', str(output)]
                completed = subprocess.run(
                    command, env=environment, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    timeout=150.0, check=False)
                if completed.returncode != 0 or not output.is_file():
                    raise RuntimeError(
                        f'canonical run {run_number} failed:\n{completed.stdout}')
                result = json.loads(output.read_text(encoding='utf-8'))
        finally:
            terminate(stack)
    if stack.returncode not in (0, -signal.SIGINT):
        raise RuntimeError(
            f'launch exited with {stack.returncode}; see {log_path}')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--seed', type=int, default=12012)
    parser.add_argument('--domain-base', type=int, default=212)
    parser.add_argument(
        '--output', type=Path,
        default=Path('benchmarks/canonical_demo_results.json'))
    parser.add_argument(
        '--csv', type=Path,
        default=Path('benchmarks/canonical_demo_results.csv'))
    args = parser.parse_args()
    if args.runs < 3:
        parser.error('Gate 12 requires at least three consecutive runs')

    root = Path(__file__).resolve().parents[1]
    probe = root / 'tools' / 'run_gate11_demo.py'
    results = []
    for run_number in range(1, args.runs + 1):
        print(f'RUN {run_number}/{args.runs} seed={args.seed}', flush=True)
        log_path = Path('/tmp') / f'canonical_demo_run_{run_number}.log'
        result = run_once(
            run_number, args.seed, args.domain_base + run_number,
            probe, log_path)
        results.append(result)
        print(
            f"RUN {run_number}: {result['result']} "
            f"duration={result['goal_duration_s']}s "
            f"goal={result['goal_status']} "
            f"reaction={result['obstacle_reaction_latency_ms']}ms",
            flush=True)

    summary = {
        'scenario': 'canonical_perception_navigation',
        'seed': args.seed,
        'runs': len(results),
        'consecutive_successes': sum(
            result['result'] == 'PASS' for result in results),
        'all_successful': all(result['result'] == 'PASS' for result in results),
        'mean': {
            metric: round(statistics.fmean(
                float(result[metric]) for result in results), 3)
            for metric in METRICS
        },
    }
    document = {'summary': summary, 'runs': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    with args.csv.open('w', newline='', encoding='utf-8') as stream:
        fields = ['run', 'result', 'goal_status', 'collision_count', *METRICS]
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
