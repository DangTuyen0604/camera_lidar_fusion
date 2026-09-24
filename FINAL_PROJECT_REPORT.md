# Final Project Report

Verification date: 2026-09-23

## PROJECT STATUS: PARTIAL

The canonical camera-LiDAR-to-navigation path passes its automated checks. The
integrated warehouse launch now combines live perception with the M01-M04
mission without simulator-truth injection into the fusion topic. Its live smoke
test passed, but a continuous M01-M04 completion run has not yet been recorded;
the project therefore remains PARTIAL rather than claiming final acceptance.

## SIMULATION: PASS

- Gazebo Harmonic starts the warehouse, OpenAMRobot, camera, LiDAR, odometry,
  clock, TF, and ROS-Gazebo bridges.
- The canonical smoke test loaded the deterministic `warehouse_map.yaml`
  (90 x 70 cells at 0.2 m/cell), without the scan-shadow artifacts present in
  the optional captured SLAM map.
- Gazebo and RViz can be enabled together from the final launch command below.

## NAVIGATION: PASS

- Nav2 receives the static map, filtered scan, odometry, TF, and dynamic
  obstacle cloud.
- The command chain is `controller_server -> velocity_smoother ->
  collision_monitor -> /cmd_vel -> Gazebo`.
- Three recorded canonical runs reached the goal with zero collisions and a
  mean minimum obstacle clearance of 1.2 m.

## PERCEPTION: PASS

- The simulation detector consumes synchronized camera images and publishes
  typed 2D detections.
- Recorded mean rates: camera 10.340 Hz, point cloud 8.913 Hz, and detections
  7.568 Hz.
- Real-image inference remains dependent on the optional ONNX model.

## FUSION: PASS

- Image, CameraInfo, and PointCloud2 inputs are timestamp-synchronized before
  calibrated projection and depth estimation.
- Fused detections preserve their source frame and include valid/invalid XYZ
  state rather than silently inventing depth.
- Recorded mean fusion rate was 7.567 Hz, mean synchronization delay was
  20.880 ms, and mean end-to-end pipeline latency was 65.892 ms.

## OBSTACLE BRIDGE: PASS

- Fused XYZ detections are validated, transformed with TF2 into `base_link`,
  expanded to class-sized footprints, and published for Nav2 and Collision
  Monitor.
- Stale obstacles expire and emit clearing data. Recorded mean expiry time was
  0.812 s; every canonical run observed obstacle expiry.

## END-TO-END: PARTIAL

- Result: 3/3 consecutive canonical runs passed with deterministic seed
  `12012`.
- Every run localized, detected a simulated worker, produced fused XYZ,
  published a bridge obstacle, exercised slowdown/stop/clear actions, and
  completed the Nav2 goal without collision.
- Evidence: [`benchmarks/canonical_demo_results.json`](benchmarks/canonical_demo_results.json)
  and [`benchmarks/canonical_demo_results.csv`](benchmarks/canonical_demo_results.csv).
- Integrated warehouse smoke evidence confirmed one `object_fusion_node`
  publisher on `/fusion/detections_3d`, separate scenario truth, a valid worker
  XYZ detection with 38 LiDAR points, a 49-point `base_link` obstacle, mission
  state `GO_TO_PICKUP`, and a Nav2 path to S1.
- A complete uninterrupted M01-M04 run with live perception remains pending.

## TEST RESULTS: PASS

Command:

```bash
./scripts/run_tests.sh
```

Result: `12 packages`, `306 tests`, `0 errors`, `0 failures`, `34 skipped`.
The skipped entries are reported by the `cppcheck` lint harness; no executable
test failed. The suite includes unit, configuration, launch, TF,
synchronization, fusion, bridge-expiry, and canonical interface smoke tests.

## BENCHMARK: PASS

Three-run canonical mean:

| Metric | Result |
|---|---:|
| Goal duration | 10.384 s |
| Camera rate | 10.340 Hz |
| Point-cloud rate | 8.913 Hz |
| Detection rate | 7.568 Hz |
| Fusion rate | 7.567 Hz |
| Synchronization delay | 20.880 ms |
| End-to-end latency | 65.892 ms |
| Obstacle reaction latency | 94.545 ms |
| Obstacle expiry | 0.812 s |
| Minimum clearance | 1.2 m |
| Collisions | 0 |

Reproduce with:

```bash
python3 benchmarks/run_canonical_demo.py --runs 3
```

## FAULT INJECTION: PASS

The deterministic seed `12014` covers five implemented faults:

- 80 ms timestamp delay: reported `DELAYED` and unhealthy against a 50 ms
  tolerance.
- Extrinsic perturbation (3 degrees, 0.05 m): projection error increased by
  7.240 px.
- Point-cloud noise (0.1 m standard deviation): measured XYZ RMS change was
  0.098 m.
- Density reduction: retained 25/100 points reproducibly.
- Lost detection: obstacles expired in all three canonical runs.

Evidence: [`experiments/gate14_core_faults.json`](experiments/gate14_core_faults.json).
Reproduce with `python3 benchmarks/run_core_faults.py --seed 12014`.

## KNOWN LIMITATIONS

- The production-matched LiDAR position is occluded by the rear chassis. The
  navigation stack intentionally masks the rear 80-degree self-reflection
  sector and uses the remaining 280 degrees from `/scan_filtered`; genuine
  360-degree coverage requires a higher mount and recalibration.
- The deterministic Gazebo detector is color-based; real images require a
  compatible ONNX model.
- KITTI data is downloaded separately, so dataset-dependent tests may skip.
- Calibration drift is detected but extrinsics are not recalibrated online.
- Obstacle footprints are class-sized point sets, not tracked 3D meshes.
- Simulation results do not replace real sensor, braking-distance, weather,
  timing, or functional-safety validation.
- RViz/Gazebo may need working GPU/OpenGL support; headless mode is available.

## FINAL LAUNCH COMMAND

```bash
cd /home/tuyen/Vinmotion/camera_lidar_fusion
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fusion_bringup bringup_sim.launch.py \
  use_rviz:=true gazebo_gui:=true mission_autostart:=true
```
