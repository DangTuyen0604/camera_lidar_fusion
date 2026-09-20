"""Lifecycle owner for one live benchmark run."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from resource_monitor import ResourceMonitor
import yaml


@dataclass(frozen=True)
class Scenario:
    name: str
    parameters: dict
    runs: int
    command: tuple


def load_scenarios(path):
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    default_command = tuple(document['launch_command'])
    return [Scenario(
        name=name, parameters=value.get('parameters', {}),
        runs=int(value.get('runs', 5)),
        command=tuple(value.get('command', default_command)),
    ) for name, value in document['scenarios'].items()]


def _terminate(process, timeout=12.0):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def run_scenario(scenario, run_number, output_dir, startup_timeout, duration):
    """Launch the real stack, collect ROS metrics, and reap its process group."""
    with tempfile.TemporaryDirectory(prefix='fusion-benchmark-') as temporary:
        metrics_file = Path(temporary) / 'metrics.json'
        env = os.environ.copy()
        env['BENCHMARK_SCENARIO'] = scenario.name
        env['BENCHMARK_PARAMETERS_JSON'] = json.dumps(scenario.parameters)
        stack = subprocess.Popen(
            scenario.command, env=env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        monitor = ResourceMonitor(stack.pid)
        monitor.start()
        started = time.monotonic()
        try:
            collector_command = [
                sys.executable, str(Path(__file__).with_name('metrics_collector.py')),
                '--output', str(metrics_file), '--duration', str(duration),
                '--startup-timeout', str(startup_timeout)]
            collector = subprocess.run(
                collector_command, env=env, check=False,
                timeout=startup_timeout + duration + 15.0)
            if collector.returncode != 0:
                raise RuntimeError(
                    f'metrics collector exited {collector.returncode} for {scenario.name}')
        finally:
            _terminate(stack)
            monitor.stop()
        if not metrics_file.is_file():
            raise RuntimeError('collector produced no metrics artifact')
        row = json.loads(metrics_file.read_text(encoding='utf-8'))
        row.update({
            'scenario': scenario.name, 'run': run_number,
            'wall_time_s': time.monotonic() - started,
            'cpu_mean_percent': monitor.cpu_mean,
            'cpu_peak_percent': monitor.cpu_peak,
            'memory_peak_mb': monitor.memory_peak_mb,
            'stack_exit_code': stack.returncode})
        if monitor.remaining_pids:
            raise RuntimeError(f'orphan processes after shutdown: {monitor.remaining_pids}')
        return row
