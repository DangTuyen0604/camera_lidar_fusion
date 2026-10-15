#!/usr/bin/env python3
"""
Compare odometry sources against MuJoCo ground truth, offline.

  python3 benchmarks/odometry_benchmark.py --seeds 10 --out benchmark_results/odometry

Each scenario drives the simulator (no ROS, faster than real time) with the
realistic sensor models of the ROS simulator (the robot's MPU-6500 IMU and
AS5040 encoders, errors drawn per seed) and logs what the robot would
publish: IMU and wheel twist at 50 Hz, as its firmware does, and ground truth.
The log is replayed through
state_estimation's ekf_replay, so the filter is the same C++ code as the
ROS node. Sources compared:

  wheel         the simulator's wheel odometry (the stack's /odom so far)
  ekf_no_accel  EKF with wheels + gyro (the usual robot_localization setup)
  ekf           EKF with wheels + gyro + accelerometer and the slip gate
  ekf_no_gate   EKF with the accelerometer but without the slip and standstill gates

--no-gyro-deadband drops the robot firmware's 0.01 rad/s gyro deadband from
the IMU model, to measure what removing it from the firmware would gain.

The robot is steered on ground truth, so every source sees the same motion.
Errors are measured in the odometry frame (the start pose), where odometry
can only drift; AMCL corrects that drift in the full stack.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

SOURCES = ('wheel', 'ekf_no_accel', 'ekf', 'ekf_no_gate')
REPLAY_OPTIONS = {'ekf_no_accel': ['--no-accel'], 'ekf': [], 'ekf_no_gate': ['--no-gate']}
TICK = 0.01            # command tick [s]; the IMU and wheels are read every 2nd one (50 Hz)
SPEED = 0.4            # cruise speed [m/s]
TURN_RATE = 0.6        # in-place turn rate [rad/s]


@dataclass
class Scenario:
    name: str
    start: tuple
    description: str


SCENARIOS = {
    'rectangle': Scenario(
        'rectangle', (-3.0, -5.5, 0.0),
        '10 m x 6 m rectangle, stop and turn in place at each corner; 2 laps '
        'counter-clockwise then 2 clockwise (UMBmark-style), ~130 m'),
    'patrol': Scenario(
        'patrol', (-3.0, -5.5, 0.0),
        'the same rectangle with rounded corners at 0.4 m/s, never stopping; '
        '3 laps, ~95 m'),
    'obstacle_push': Scenario(
        'obstacle_push', (6.0, -4.0, 0.0),
        'drive into the east wall (2 m) at 0.3 m/s and keep pushing with the '
        'wheels slipping, back off 2 m; pushes of 3 s, 3 s and 6 s'),
    'spin': Scenario(
        'spin', (0.0, -5.0, 0.0),
        '3 turns in place each way at 0.6 rad/s with stops (wheel scrub)'),
}

RECTANGLE = [(-3.0, -5.5), (7.0, -5.5), (7.0, 0.5), (-3.0, 0.5)]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Driver:
    """Command generators steering on ground truth; each yields (v, w) per control step."""

    def __init__(self, sim):
        self.sim = sim

    def pose(self):
        return self.sim.robot_pose()

    def stop(self, seconds):
        for _ in range(int(seconds / (2 * TICK))):
            yield 0.0, 0.0

    def turn_to(self, heading):
        while True:
            error = wrap(heading - self.pose()[2])
            if abs(error) < 0.01:
                break
            yield 0.0, float(np.clip(2.0 * error, -TURN_RATE, TURN_RATE))
        yield from self.stop(1.0)

    def drive_to(self, goal):
        while True:
            x, y, yaw = self.pose()
            dx, dy = goal[0] - x, goal[1] - y
            distance = math.hypot(dx, dy)
            if distance < 0.03:
                break
            error = wrap(math.atan2(dy, dx) - yaw)
            v = min(SPEED, 0.8 * distance + 0.05)
            yield v, float(np.clip(1.5 * error, -0.5, 0.5))
        yield from self.stop(1.0)

    def drive_until_blocked(self, speed, timeout=15.0):
        """Drive straight until the robot stops making progress (contact)."""
        history = []
        for _ in range(int(timeout / (2 * TICK))):
            x, y, _ = self.pose()
            history.append((x, y))
            recent = history[-10:]  # 0.2 s
            if len(history) > 50 and math.dist(recent[0], recent[-1]) < 0.005:
                return
            yield speed, 0.0
        raise RuntimeError('no contact within the timeout')

    def waypoints(self, points):
        for point in points:
            x, y, _ = self.pose()
            yield from self.turn_to(math.atan2(point[1] - y, point[0] - x))
            yield from self.drive_to(point)

    def pursue(self, points, laps, lookahead=0.8):
        """Pure pursuit around a closed polygon without stopping (rounded corners)."""
        path = densify(points + [points[0]], 0.05)
        index = 0
        total = laps * (len(path) - 1)
        while index < total:
            x, y, yaw = self.pose()
            window = [path[(index + k) % (len(path) - 1)] for k in range(60)]
            nearest = int(np.argmin([math.hypot(px - x, py - y) for px, py in window]))
            index += nearest
            target = path[(index + int(lookahead / 0.05)) % (len(path) - 1)]
            c, s = math.cos(yaw), math.sin(yaw)
            tx = c * (target[0] - x) + s * (target[1] - y)
            ty = -s * (target[0] - x) + c * (target[1] - y)
            curvature = 2.0 * ty / max(tx * tx + ty * ty, 1e-6)
            yield SPEED, float(np.clip(SPEED * curvature, -1.0, 1.0))
        yield from self.stop(1.0)


def densify(points, step):
    out = []
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n)]
    out.append(points[-1])
    return out


def commands(name, driver):
    if name == 'rectangle':
        yield from driver.stop(5.0)  # docked at start: the gyro bias is learned here
        for _ in range(2):
            yield from driver.waypoints(RECTANGLE[1:] + RECTANGLE[:1])
        for _ in range(2):
            yield from driver.waypoints(list(reversed(RECTANGLE[1:])) + RECTANGLE[:1])
    elif name == 'patrol':
        yield from driver.stop(5.0)
        yield from driver.pursue(RECTANGLE, laps=3)
    elif name == 'obstacle_push':
        yield from driver.stop(5.0)
        for push in (3.0, 3.0, 6.0):
            yield from driver.drive_until_blocked(0.3)
            for _ in range(int(push / (2 * TICK))):
                yield 0.3, 0.0           # pushing, wheels slipping
            for _ in range(int(2.0 / 0.3 / (2 * TICK))):
                yield -0.3, 0.0
            yield from driver.stop(1.0)
    elif name == 'spin':
        yield from driver.stop(5.0)
        for direction in (1.0, -1.0):
            for _ in range(3):
                for _ in range(int(2 * math.pi / TURN_RATE / (2 * TICK))):
                    yield 0.0, direction * TURN_RATE
                yield from driver.stop(1.0)
    else:
        raise ValueError(name)


def record(scenario, seed, sim_factory):
    """Drive one scenario; return the sensor log lines and the truth/wheel arrays."""
    sim = sim_factory(seed)
    sim.reset(*scenario.start)
    sim.step(round(0.5 / sim.model.opt.timestep))  # settle on the floor
    sim.imu_sample()  # start the IMU averaging window here
    origin = np.array(sim.robot_pose())
    odom_origin = sim.odom.copy()
    steps = round(TICK / sim.model.opt.timestep)
    log, rows = [], []
    travel = sim.odom_travel.copy()
    tick = 0
    for v, w in commands(scenario.name, Driver(sim)):
        sim.set_cmd_vel(v, w)
        for _ in range(2):
            sim.step(steps)
            tick += 1
            t = tick * TICK
            if tick % 2 == 0:
                # The firmware reads both in one control cycle and publishes them together.
                _, gyro, accel = sim.imu_sample()
                log.append(f'imu,{t:.3f},{gyro[2]:.7f},{accel[0]:.6f},{gyro[1]:.7f}')
                twist = (sim.odom_travel - travel) / (2 * TICK)
                travel = sim.odom_travel.copy()
                log.append(f'wheel,{t:.3f},{twist[0]:.6f},{twist[1]:.6f}')
                x, y, yaw = sim.robot_pose()
                rows.append((t, x, y, yaw, *(sim.odom - odom_origin), v))
    data = np.array(rows)
    # Truth in the start (odometry) frame.
    c, s = math.cos(origin[2]), math.sin(origin[2])
    dx, dy = data[:, 1] - origin[0], data[:, 2] - origin[1]
    truth = np.stack([data[:, 0], c * dx + s * dy, -s * dx + c * dy,
                      np.unwrap(data[:, 3]) - origin[2]], axis=1)
    wheel = np.stack([data[:, 0], data[:, 4], data[:, 5], np.unwrap(data[:, 6])], axis=1)
    errors = {'gyro_bias_after_boot_calibration_deg_s':
              math.degrees(sim.gyro_bias[2] - sim.gyro_offset[2]),
              'accel_bias_mg': sim.accel_bias[0] / 9.80665 * 1000,
              'radius_error_pct': (sim.wheel_radius_error * 100).tolist(),
              'track_error_pct': sim.track_error * 100}
    return log, truth, wheel, errors


def replay(binary, log_path, out_path, options):
    completed = subprocess.run([str(binary), str(log_path), str(out_path), *options],
                               capture_output=True, text=True, check=True)
    estimate = np.genfromtxt(out_path, delimiter=',', names=True)
    return np.stack([estimate['t'], estimate['x'], estimate['y'],
                     np.unwrap(estimate['yaw'])], axis=1), completed.stderr.strip()


def metrics(truth, estimate):
    """Errors of estimate (t, x, y, yaw) against truth at truth's timestamps."""
    x = np.interp(truth[:, 0], estimate[:, 0], estimate[:, 1])
    y = np.interp(truth[:, 0], estimate[:, 0], estimate[:, 2])
    yaw = np.interp(truth[:, 0], estimate[:, 0], estimate[:, 3])
    position = np.hypot(x - truth[:, 1], y - truth[:, 2])
    heading = np.degrees(np.abs(np.arctan2(np.sin(yaw - truth[:, 3]),
                                           np.cos(yaw - truth[:, 3]))))
    distance = float(np.sum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2]))))
    return {'final_position_m': float(position[-1]),
            'max_position_m': float(position.max()),
            'rmse_position_m': float(np.sqrt(np.mean(position ** 2))),
            'final_heading_deg': float(heading[-1]),
            'max_heading_deg': float(heading.max()),
            'distance_m': distance,
            'final_position_pct': 100.0 * float(position[-1]) / max(distance, 1e-9)}


def summarize(results):
    table = {}
    for scenario in SCENARIOS:
        for source in SOURCES:
            runs = [r['metrics'][source] for r in results
                    if r['scenario'] == scenario and source in r['metrics']]
            if not runs:
                continue
            table[(scenario, source)] = {
                key: (float(np.mean([r[key] for r in runs])),
                      float(np.percentile([r[key] for r in runs], 90)),
                      float(np.max([r[key] for r in runs])))
                for key in runs[0]}
    return table


def markdown(table, seeds):
    lines = [f'Mean / 90th percentile / max over {seeds} seeds (sensor error draws).', '',
             '| scenario | source | final position error [m] | max position error [m] | '
             'final heading error [deg] |',
             '|---|---|---|---|---|']
    for (scenario, source), row in table.items():
        def cell(key, digits):
            mean, p90, worst = row[key]
            return f'{mean:.{digits}f} / {p90:.{digits}f} / {worst:.{digits}f}'
        lines.append(f'| {scenario} | {source} | {cell("final_position_m", 3)} | '
                     f'{cell("max_position_m", 3)} | {cell("final_heading_deg", 2)} |')
    return '\n'.join(lines)


def find_replay():
    from ament_index_python.packages import get_package_prefix
    return Path(get_package_prefix('state_estimation')) / 'lib' / 'state_estimation' / 'ekf_replay'


def sim_factory_for_repository(gyro_deadband=True):
    src = Path(__file__).resolve().parents[1] / 'src'
    sys.path.insert(0, str(src / 'warehouse_mujoco'))
    from warehouse_mujoco.simulator import EncoderConfig, ImuConfig, WarehouseSim
    imu = ImuConfig.mpu6500()
    if not gyro_deadband:
        imu.gyro_deadband = 0.0

    def make(seed):
        return WarehouseSim(src / 'warehouse_mujoco/mjcf/scene.xml',
                            src / 'warehouse_simulation/worlds/warehouse.sdf',
                            [src / 'warehouse_simulation/models'],
                            src / 'openamrobot_description/meshes/visual', seed=seed, imu=imu,
                            encoder=EncoderConfig.as5040())
    return make


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--first-seed', type=int, default=1)
    parser.add_argument('--scenarios', nargs='+', default=list(SCENARIOS),
                        choices=list(SCENARIOS))
    parser.add_argument('--out', type=Path, default=Path('benchmark_results/odometry'))
    parser.add_argument('--no-gyro-deadband', action='store_true',
                        help="drop the firmware's 0.01 rad/s gyro deadband (measures its effect)")
    parser.add_argument('--replay', type=Path, default=None,
                        help='ekf_replay binary (default: from the sourced workspace)')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    binary = args.replay or find_replay()
    make_sim = sim_factory_for_repository(gyro_deadband=not args.no_gyro_deadband)

    results = []
    for scenario_name in args.scenarios:
        scenario = SCENARIOS[scenario_name]
        for seed in range(args.first_seed, args.first_seed + args.seeds):
            started = time.monotonic()
            log, truth, wheel, errors = record(scenario, seed, make_sim)
            log_path = args.out / f'{scenario_name}_{seed}.log.csv'
            log_path.write_text('\n'.join(log) + '\n')
            run = {'scenario': scenario_name, 'seed': seed, 'errors': errors,
                   'metrics': {'wheel': metrics(truth, wheel)}, 'replay': {}}
            for source, options in REPLAY_OPTIONS.items():
                estimate, stderr = replay(binary, log_path,
                                          args.out / f'{scenario_name}_{seed}_{source}.csv',
                                          options)
                run['metrics'][source] = metrics(truth, estimate)
                run['replay'][source] = stderr
            np.savetxt(args.out / f'{scenario_name}_{seed}_truth.csv', truth, delimiter=',',
                       header='t,x,y,yaw', comments='', fmt='%.5f')
            results.append(run)
            summary = ', '.join(f"{s} {run['metrics'][s]['final_position_m']:.3f} m"
                                for s in SOURCES if s in run['metrics'])
            print(f'{scenario_name} seed {seed}: {summary} '
                  f'({time.monotonic() - started:.0f} s)', flush=True)

    table = summarize(results)
    report = markdown(table, args.seeds)
    (args.out / 'results.json').write_text(json.dumps(
        {'scenarios': {k: v.description for k, v in SCENARIOS.items()},
         'runs': results}, indent=1))
    (args.out / 'summary.md').write_text(report + '\n')
    print(report)


if __name__ == '__main__':
    main()
