# Wheel + IMU odometry (EKF)

`src/state_estimation` fuses the wheel encoders with the 6-axis IMU into the
robot's odometry (`/odom` and the `odom -> base_footprint` TF). AMCL corrects
odometry against the map; the better the odometry, the less it has to
correct, and the less the robot drifts between scan matches or while AMCL is
uncertain. The project had one concrete case: the robot pressed into a low
pallet, its wheels kept turning, and the wheel odometry "drove" through the
pallet until AMCL drifted by up to 50 cm.

```
/wheel/odom (twist, 50 Hz) ─┐
                            ├─► WheelImuEkf ─► /odom, TF odom -> base_footprint, /diagnostics
/imu (gyro + accel, 50 Hz) ─┘
```

`ros2 launch fusion_bringup final_demo.launch.py odometry:=ekf sensor_errors:=realistic`
starts it; `odometry:=wheel` (default) keeps the simulator's wheel odometry.

## Filter

State `[x, y, yaw, v, b_gz, b_a, pitch, b_gy]`: planar pose, forward speed,
z and y gyro biases, forward accelerometer bias, chassis pitch. The IMU drives
the prediction (50 Hz on the robot):

```
yaw'   = yaw + (w_z - b_gz) dt
pitch' = pitch + (w_y - b_gy) dt
v'     = v + (f_x - b_a + g sin(pitch)) dt
p'     = p + (v dt + a dt^2/2) [cos, sin](yaw + w dt / 2)
```

and the wheels correct it (50 Hz):

| measurement | when | why |
|---|---|---|
| wheel speed `v_wheel = v` | always, unless rejected as slip | makes `b_a` observable; rejecting it is how slip is handled |
| level floor `pitch = 0 +- 0.2 deg` | when the wheel speed is accepted | anchors pitch; `b_a` takes the static offset |
| zero angular rate `w_z - b_gz = 0`, `w_y - b_gy = 0` | wheels still for 0.25 s | learns the gyro biases |

The C++ core (`wheel_imu_ekf.cpp`) has no ROS dependency; the node, the unit
tests and the offline replay tool (`ekf_replay`) all use the same class.
Covariance updates use the Joseph form. One predict + one update take about
1 µs each (`ekf_replay` timing, RelWithDebInfo).

## Design decisions, and the evidence for each

Each of these came from a failure seen in simulation, and each has a unit
test that fails when the mechanism is removed. The numbers in this list were
measured during development, with an earlier and better IMU model (Bosch
BMI088 noise figures at 100 Hz, no firmware processing); the evaluation
below uses the robot's own sensors.

1. **Heading from the gyro alone; the wheel yaw rate only at standstill.**
   While the robot turns, a gyro bias and a wheel track/scrub error are not
   separable. Fused as white noise, the systematic wheel yaw error made the
   filter learn a wrong bias that reproduced the wheel heading error.
   (`GyroCorrectsWheelHeadingErrorOnACircle`)
2. **Zero angular-rate updates need 0.25 s of standstill and pass a
   chi-square gate.** When the wheels stop after an in-place turn the chassis
   rocks back at up to 6 deg/s for ~0.3 s. One update in that window moved
   the bias, and through the heading-bias correlation (about -60 s after a
   minute without standstill) "corrected" the heading by 1.8 deg; over a
   spin test the heading error grew from 0.2 deg to 5 deg.
   (`IgnoresChassisRockingWhenTheWheelsStop`)
3. **Wheel speed gated, with a 0.1 m/s floor and a 5 s resynchronization.**
   A wheel speed whose normalized innovation exceeds the chi-square gate
   (p = 0.001) is rejected as slip. Without the floor, the small
   wheel/IMU disagreement while accelerating (~0.08 m/s in MuJoCo) started
   rejection runs that locked the filter on a wrong speed (final error 2.2 m
   instead of 2 cm in the push scenario). After 5 s of continuous rejection
   the speed (and the accelerometer bias) covariance is reopened, so an IMU
   that diverged cannot lock the gate forever; 2 s, the first value, ended
   ordinary 3 s pushes that way. (`AcceptsSmallWheelImuDisagreementWhileAccelerating`,
   `RecoversAfterAPushWithoutResynchronizing`, `ResynchronizesWhenTheImuSpeedDiverges`)
4. **Gravity compensation with a pitch state.** Pushing a wall, the chassis
   pitches by ~0.2 deg; that leaks 0.04 m/s^2 of gravity into the forward
   accelerometer, the IMU-only speed drifts, and the gate opens to the
   slipping wheels after ~1.8 s. A Gauss-Markov pitch decaying to zero was
   tried first and diverged: it forgot the pitch of a sustained push, and its
   driving noise inflated the speed variance until the gate opened. The final
   model integrates the y gyro and applies the level-floor constraint only
   while the wheels are trusted. Push scenario: 1.9 m -> 0.02 m.
   (`RecoversAfterAPushWithoutResynchronizing` with `gravity = 0`)

## Simulated sensors

`sensor_errors:=realistic` (`warehouse_mujoco/simulator.py`) models the
sensors of the real OpenAMRobot, the robot this model comes from: its
documentation and firmware ([openamr-platform-sw](https://github.com/openAMRobot/openamr-platform-sw),
[openamr-platform-fw](https://github.com/openAMRobot/openamr-platform-fw))
list an InvenSense MPU-6500 IMU and AMS AS5040 wheel encoders, both read by
a Teensy at 50 Hz. The real robot's odometry fuses the wheels with the IMU's
z gyro in robot_localization; this filter does the same, plus the
accelerometer for slip.

| part | model | source |
|---|---|---|
| gyro (chip) | 0.01 deg/s/sqrt(Hz) noise, turn-on offset N(0, 2.5 deg/s), scale error N(0, 1.5 %), bias random walk 1.4e-4 rad/s/sqrt(s) | MPU-6500 datasheet (PS-MPU-6500A-01 rev 1.3): rate noise density, initial ZRO +-5 deg/s and sensitivity +-3 % (taken as 2 sigma); the random walk assumes a ~2 K/h warm-up at the datasheet's 0.24 deg/s/K |
| accelerometer (chip) | 300 ug/sqrt(Hz), turn-on offset N(0, 30 mg), random walk 2.1e-4 m/s^2/sqrt(s) | MPU-6500: noise density, zero-g +-60 mg; 0.64 mg/K for the random walk |
| IMU firmware | +-250 deg/s and +-2 g quantization; gyro offset averaged over 40 samples at boot and subtracted; gyro axes below 0.01 rad/s (0.57 deg/s) read 0 | linorobot2_hardware `IMUInterface` / MPU9250 driver used by openamr-platform-fw |
| IMU sampling | 50 Hz, each sample the mean over its period (dv/dt of the IMU site, mean angular rate) | rate: the firmware's 20 ms control loop. Averaging: MuJoCo's instantaneous accelerometer is dominated by contact chatter (+-12 m/s^2, 0.21 correlation with the true acceleration) |
| wheel encoders | 1024 counts per wheel revolution (one count per 20 ms is 0.034 m/s), wheel radius error N(0, 0.2 %) per wheel, track width error N(0, 1 %) | counts: `COUNTS_PER_REV` in the firmware; the calibration errors are assumed for an uncalibrated robot (UMBmark's E_d, E_b) |

The default (`ideal`) keeps the previous behaviour, so existing tests and
results are unchanged.

Not modelled: the AS5040's per-revolution ripple (+-40 % before the
firmware's per-boot correction table, +-4 % after), and the real robot's
geometry. Its firmware uses a 0.10 m wheel radius and 0.46 m track; this
simulation, like the robot description it was built from, uses 0.11 m and
0.4075 m.

## Evaluation

`benchmarks/odometry_benchmark.py` drives MuJoCo directly (no ROS, ~10x real
time), logs exactly what the robot would publish, and replays the log through
`ekf_replay`, so the numbers come from the same C++ filter as the node. Each
seed draws new sensor errors. Errors are in the odometry frame (from the start
pose); AMCL is not involved.

| source | meaning |
|---|---|
| `wheel` | the simulator's wheel odometry, the stack's `/odom` so far |
| `ekf_no_accel` | wheels + gyro (the usual robot_localization setup) |
| `ekf` | this filter |
| `ekf_no_gate` | this filter without the slip and standstill gates |

Results, 10 seeds per scenario. Each cell is mean / 90th percentile / max
of the final position error, and the mean final heading error. "Better in"
counts the seeds where the EKF ended closer to the truth than the wheels.

**The robot as it is** (MPU-6500 behind its firmware, 0.01 rad/s gyro deadband):

| scenario (distance) | wheel odometry | EKF | EKF better in | EKF, wheels + gyro only |
|---|---|---|---|---|
| rectangle, stops at corners (128 m) | 4.51 / 8.05 / 10.0 m, 51 deg | 3.53 / 5.92 / 7.57 m, 37 deg | 7/10 | 3.53 m |
| patrol, never stops (93 m) | 3.85 / 7.19 / 8.85 m, 43 deg | 2.77 / 4.51 / 4.82 m, 28 deg | 5/10 | 2.77 m |
| pushing a wall, wheels slipping (11 m) | 3.03 / 3.03 / 3.04 m | 1.03 / 1.28 / 1.45 m | 10/10 | 3.02 m |
| spinning in place | 0.27 deg | 2.58 deg | 3/10 | 2.58 deg |

**Without the firmware deadband** (`--no-gyro-deadband`, everything else the same):

| scenario | wheel odometry | EKF | EKF better in |
|---|---|---|---|
| rectangle | 4.51 m, 51 deg | 0.20 / 0.37 / 0.42 m, 3.3 deg | 10/10 |
| patrol | 3.85 m, 43 deg | 3.02 / 4.47 / 5.90 m, 31 deg | 5/10 |
| pushing a wall | 3.03 m | 0.88 / 1.23 / 1.27 m | 10/10 |
| spinning in place | 0.27 deg | 0.46 deg | 4/10 |

What the numbers say:

- **Slip is where the filter clearly wins** (10/10 seeds, 3 m -> ~1 m). The
  accelerometer is what does it: the same filter without it (wheels + gyro
  only, the robot_localization setup the real robot uses) does nothing for
  slip. What remains is the 6 s push, which outlasts the 5 s rejection
  limit, and pitch drift of the y gyro during the 3 s pushes.
- **The firmware deadband is the largest single error.** It zeroes not only
  slow rotation but the gyro bias at rest, so the standstill update never
  sees the bias drift (up to +-0.4 deg/s over the 8 min rectangle) and the
  heading absorbs it during every turn. Removing it takes the rectangle from
  3.5 m to 0.2 m. Ablation over 5 seeds of the rectangle with the deadband:
  3.24 m with every error source, 0.42 m without the gyro bias random walk.
- **Gyro scale error limits one-way driving.** On the patrol (three laps in
  one direction) the EKF beats the wheels in only half the seeds, with or
  without the deadband. Ablation over 5 seeds without the deadband: 2.88 m
  with every error, 0.68 m without the +-3 % scale error, 0.09 m without
  scale error and bias walk. Neither is observable from these sensors; a
  one-time gyro scale calibration (turns against a known heading) would fix
  the first.
- Two inputs are assumptions, not datasheet values, and they drive these
  results: the gyro bias random walk (a 2 K/h warm-up at the datasheet's
  worst-case 0.24 deg/s/K) and the wheel calibration errors.

Tried and removed: while the gyro sat in the deadband (reading exactly 0),
propagating the heading with the wheels' yaw rate instead. It targeted the
slow rotations the deadband hides, but those were not the problem: the
wheels' yaw is itself biased by the radius and track errors while driving
straight, and the filter inherited that. Over 5 seeds of the rectangle it
was worse wherever the bias drift was small (2.21 m against 0.42 m without
the bias walk).

Recommendations for the real robot, in order of effect: remove (or shrink
well below 0.01 rad/s) the firmware gyro deadband, since at 50 Hz the
MPU-6500's noise is ~0.05 deg/s and the standstill update handles the bias;
calibrate the gyro scale once; fuse the accelerometer for slip.

## Live stack

With `odometry:=ekf sensor_errors:=realistic`, `/odom` comes from the EKF at
the IMU rate and the simulator's wheel odometry moves to `/wheel/odom`
without a TF. A recorded Nav2 run (one goal, 4.7 m, with the earlier BMI088
model) replayed offline gives the same trajectory as the live node within
1 cm, and its error against ground truth was 3.5 cm. Multi-run results in the full stack are part of the
scenario benchmark (docs/benchmark.md).

## Limitations

- **Gyro scale error is not estimated.** It is not observable from these
  sensors (the wheels' yaw is unreliable while turning). With the MPU-6500's
  +-3 % sensitivity it dominates one-way driving: on the patrol the EKF is
  no better than the wheels in half the seeds. AMCL has to correct it, or a
  one-time scale calibration.
- **Under the robot's firmware deadband the gyro bias is invisible at
  rest,** so its drift is not tracked (see Evaluation). The filter cannot fix
  this from the outside; the firmware can.
- **Slip detection has limits.** Slips slower than 0.1 m/s are not rejected,
  and a slip longer than 5 s ends in a resynchronization to the slipping
  wheels (the 6 s push in the benchmark).
- **The vibration model is tuned for a 50 Hz IMU stream.** Driving straight
  at 0.35 m/s in MuJoCo, the accelerometer still carries 0.46 m/s^2 rms of
  contact chatter at 50 Hz and 0.76 at 100 Hz, far above `accel_vibration`
  (0.05). At 100 Hz the gate rejected good wheel speeds as slip during a
  full mission and the position ran off until AMCL lost the robot; the
  simulator now publishes 50 Hz in both sensor modes, and then the mission's
  map->base yaw error is p95 0.44 deg (wheel odometry: 1.38). Raising
  `accel_vibration` instead is not a fix: at 0.5 the wall push ends 2.78 m
  off instead of 1.13 m, as bad as the wheels, because slip then looks like
  vibration.
- **Simulation only.** The error models follow the robot's datasheets and
  firmware, but the chassis dynamics (rocking, pitch while pushing) are
  MuJoCo's, and the simulated robot's wheel radius and track differ from the
  real one's. KITTI was considered
  for real-data validation and rejected: its OXTS data are the outputs of an
  INS that is already filtered, and it has no wheel encoders, so it cannot
  exercise wheel/IMU fusion or slip rejection.
- Lever arm and roll are ignored (the IMU sits on the rotation axis; the
  floor is level).
