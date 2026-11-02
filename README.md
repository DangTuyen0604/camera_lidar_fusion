# Camera–LiDAR Fusion for AMR Person Safety

[![Build and test](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/build-and-test.yml)
[![Lint](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/lint.yml/badge.svg)](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/lint.yml)

A warehouse robot drives toward a goal. A worker steps into the aisle. The
camera finds the worker, the LiDAR measures how far away they are, and the
fused 3D position becomes an obstacle in the Nav2 costmaps. The Collision
Monitor, which also watches the planar scan, slows the robot. When the worker
leaves, the obstacle expires and the robot finishes its goal.

Everything runs in ROS 2 Jazzy with a MuJoCo warehouse simulation. The fusion
core is C++ (Eigen, TF2); the same code also runs on KITTI recordings.

```text
camera ─► detector ─┐
                    ├─► sync ─► LiDAR→image projection ─► depth per box ─► fused XYZ
LiDAR ──────────────┘                                                         │
                                                                              ▼
  MuJoCo ◄── /cmd_vel ◄── Collision Monitor ◄── Nav2 costmaps ◄── obstacle bridge
```

## Results

Canonical scenario (`benchmarks/run_canonical_demo.py`, seed 12012, 10 runs
per detector, `runtime-cpu` Docker image with NVIDIA rendering): the robot is
given a Nav2 goal, a worker is spawned 1.2 m ahead only after it starts moving,
then removed once the robot has reacted. `color` is the deterministic
simulation detector; `yolo` is the YOLOv8n trained on warehouse images
(`models/best.onnx`: person, box, pallet), which also detects the boxes on the
shelves.

| Metric: median (range over 10 runs) | `color` | `yolo` |
|---|---:|---:|
| Runs that reached the goal without collision | 10/10 | 10/10 |
| Worker spawned → camera detection fused into a 3D position | 172 ms (133–262) | 271 ms (149–436) |
| Worker spawned → Collision Monitor slows the robot | 102 ms (50–140) | 103 ms (0–158) |
| Sensor timestamp → fused detection published | 65 ms | 106 ms |
| Camera–LiDAR timestamp offset | 20 ms | 20 ms |
| Worker removed → bridge publishes an empty obstacle cloud | 0.80 s (0.75–0.90) | 1.99 s (1.84–2.25) |
| LiDAR points on the worker | 23–24 | 24–25 |
| Camera / point cloud / fusion rate | 11.2 / 8.9 / 7.9 Hz | 9.1 / 7.4 / 5.6 Hz |
| Closest approach to the worker | 1.2 m | 1.2 m |

The Collision Monitor takes both the planar scan and the fused obstacles. In
10/10 `yolo` runs and 9/10 `color` runs it slowed the robot before the camera
had fused the worker, so that reaction time belongs to the planar LiDAR, not to
the fusion pipeline; the fastest `yolo` reactions (0.4 and 16 ms) are too quick
to have been caused by the worker at all. Every reaction was the slow zone's
slowdown, and the robot never came closer to the worker than where it was when
the worker appeared. What fusion adds is the class and a class-sized footprint
in the costmaps.

Raw per-run data: [`canonical_demo_results.json`](benchmarks/canonical_demo_results.json)
(`color`) and [`canonical_demo_results_yolo.json`](benchmarks/canonical_demo_results_yolo.json)
(`yolo`).

Two supporting results:

- **Obstacles below the scan plane.** A 0.18 m pallet is invisible to the
  0.46 m-high planar LiDAR. Adding a simulated 16-ring LiDAR with C++ ground
  removal and a 3D costmap layer took full warehouse missions from robot–pallet
  contact in 2/4 runs to 0/3. In the latest 4 missions (3 wheel odometry,
  1 EKF) the robot kept 25–66 cm from it
  ([docs/pointcloud_preprocess.md](docs/pointcloud_preprocess.md)).
- **Localization that arrived 1 s late.** Against simulator ground truth over
  full missions, AMCL's own pose stayed within 0.3° (p95), yet the
  map→robot transform the costmaps use was off by 3.5° (p95, up to 7°) after
  the robot reversed a turn. AMCL's default 1 s `transform_tolerance`
  future-dates each correction, so every consumer applied it a second late
  and obstacles were smeared across the map. At 0.2 s: 1.4–1.5° (p95).
- **Preprocessing speed.** Profiling the C++ ground removal and clustering on a
  131k-point synthetic HDL-64 scan cut it from 1645 ms to 21.4 ms per frame
  (build type, hash function, neighbour caching). The scan is synthetic, not
  KITTI.

## How it works

| Step | What happens | Code |
|---|---|---|
| 1. Synchronize | Image, CameraInfo and PointCloud2 are matched with ROS `ApproximateTime`; a triple whose stamps spread wider than the tolerance is dropped and reported on `/fusion/sync_status`. | [`sensor_sync_node.cpp`](src/perception_core/src/nodes/sensor_sync_node.cpp), [`timestamp_sync.cpp`](src/perception_core/src/core/timestamp_sync.cpp) |
| 2. Detect | 2D boxes from YOLOv8 (ONNX Runtime) on real images, or from a deterministic color detector in simulation. | [`yolo_detector/`](src/yolo_detector/yolo_detector) |
| 3. Project | Each LiDAR point is moved into the camera frame with TF2 at the image timestamp and projected: `X_c = R_rect · T_cam←lidar · X_l`, `[u v 1]ᵀ ∝ P · X_c`. Points behind the camera or outside the image are dropped. | [`lidar_projector.cpp`](src/perception_core/src/core/lidar_projector.cpp) |
| 4. Depth per box | Keep points inside the central part of the box, sort by depth, split into clusters at depth gaps, take the nearest cluster with enough points, reject outliers with the median absolute deviation, and report the per-axis median as the object position. Too few points → `valid=false`, never a made-up depth. | [`object_depth_estimator.cpp`](src/perception_core/src/core/object_depth_estimator.cpp), [`object_fusion_node.cpp`](src/perception_core/src/nodes/object_fusion_node.cpp) |
| 5. Obstacle bridge | Reject invalid, stale (> 0.5 s), future-dated or untransformable detections; transform to `base_link`; publish a class-sized footprint as PointCloud2. After 0.75 s without a detection, publish clearing rays so no ghost obstacle stays behind. | [`detection_obstacle_bridge_node.cpp`](src/navigation_bridge/src/detection_obstacle_bridge_node.cpp) |
| 6. Nav2 | The obstacle cloud feeds both costmaps and the Collision Monitor. Commands flow `controller_server → velocity_smoother → collision_monitor → /cmd_vel`, so the Collision Monitor has the last word. | [`nav2_canonical_params.yaml`](src/navigation_bringup/config/nav2_canonical_params.yaml) |

More detail: [projection math](docs/projection_math.md),
[object fusion](docs/object_fusion.md),
[timestamp synchronization](docs/timestamp_synchronization.md),
[dynamic obstacles](docs/dynamic_obstacles.md),
[person safety](docs/person_safety.md).

## Run it

### Ubuntu 24.04 with ROS 2 Jazzy

```bash
./scripts/install_dependencies.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# The canonical person-safety scenario (MuJoCo viewer + RViz + camera window)
ros2 launch fusion_bringup final_demo.launch.py mission:=false

# Reproduce the results table (headless, 10 runs per detector)
python3 benchmarks/run_canonical_demo.py --runs 10 --keep-going
python3 benchmarks/run_canonical_demo.py --runs 10 --keep-going \
  --detector yolo --model-path models/best.onnx \
  --output benchmarks/canonical_demo_results_yolo.json \
  --csv benchmarks/canonical_demo_results_yolo.csv
```

### Any other Linux host (Docker)

```bash
scripts/run_docker_gui.sh ros2 launch fusion_bringup final_demo.launch.py mission:=false
```

Add `--gpu` for NVIDIA rendering. Details in [docs/docker.md](docs/docker.md).

### Other entry points

| Command | What it shows |
|---|---|
| `ros2 launch fusion_bringup final_demo.launch.py` | The full warehouse: four-station pick-and-place missions (M01–M04) with docking, the worker scenario, a pallet that blocks an aisle, and live fusion. Add `odometry:=ekf` for the wheel + IMU EKF |
| `ros2 launch fusion_bringup yolo_demo.launch.py` | The same missions with a YOLOv8n trained on warehouse images (`models/best.onnx`, not in the repository, see [models/README.md](models/README.md)) |
| `ros2 launch fusion_bringup projection_demo.launch.py use_rviz:=true` | LiDAR projected onto KITTI images (`./tools/download_kitti.sh` first) |
| `./scripts/run_perception_demo.sh` | YOLOv8 ONNX + fusion on KITTI (needs `models/yolov8n-opencv.onnx`, see [models/README.md](models/README.md)) |

## Tests

```bash
./scripts/run_tests.sh
```

Unit tests (gtest/pytest) cover the projection, depth estimation,
synchronization, calibration loading, PointCloud2 parsing, ground removal,
clustering and the obstacle bridge (stale, future and non-finite inputs,
unknown frames, expiry). Launch tests start the simulation and check the
topic, node and TF contract of the canonical stack and the perception
pipeline. The remaining `system_tests` check that the installed configuration
keeps the safety settings (Collision Monitor zones, worker and pallet events,
docking tolerances). KITTI-dependent tests skip when the dataset is not
downloaded. The full M01–M04 mission test (`test_warehouse_end_to_end`) is
slow and is run on demand; the CI step meant for it currently selects no
test.

## Repository layout

Main path:

| Package | Role |
|---|---|
| `perception_core` | C++: synchronization, projection, depth estimation, fusion node, point-cloud preprocessing, metrics |
| `yolo_detector` | ONNX YOLOv8 detector and the simulation color detector |
| `navigation_bridge` | C++: fused detections → validated, expiring Nav2 obstacles |
| `navigation_bringup` | AMCL, Nav2, costmaps, velocity smoother, Collision Monitor |
| `fusion_bringup` | Launch files and perception parameters |
| `fusion_interfaces` | Detection, fusion, sync and metrics messages |

Supporting:

| Package | Role |
|---|---|
| `warehouse_mujoco`, `warehouse_simulation` | MuJoCo physics, sensors, warehouse world, scripted workers and pallets |
| `openamrobot_description` | Robot URDF and TF frames |
| `warehouse_mission_manager` | M01–M04 missions, docking, yielding to people |
| `state_estimation` | C++ wheel + IMU odometry EKF with slip rejection (`odometry:=ekf`, [docs/state_estimation.md](docs/state_estimation.md)) |
| `kitti_ros2_player` | KITTI playback and calibration |
| `system_tests` | Launch-level tests |

`benchmarks/` holds the scenario probes and results, `training/` the YOLO
training scripts, `tools/` data and calibration utilities.

## Limitations

- **Simulation only.** No real robot or real sensor recording has been run
  through the full loop.
- **The YOLO weights are not in the repository.** Both detectors are
  benchmarked above, but the trained YOLOv8n (`models/best.onnx`) is not
  committed, so the `yolo` column cannot be reproduced from a fresh clone;
  the `color` column can. On KITTI the COCO-pretrained model is used.
- **Fusion accuracy is not yet measured.** The numbers above are rates and
  latencies; the fused position has not been compared against ground truth
  (e.g. KITTI 3D labels).
- **No tracking.** Each frame is fused independently and obstacles carry no
  velocity, so the robot reacts to where a person is, not where they are going.
- **Small sample.** The canonical results are 10 runs per detector of one
  scenario (seed 12012), with the worker always spawned 1.2 m ahead.
- The simulation runs fusion with relaxed settings (120 ms sync tolerance,
  ≥ 1 LiDAR point per box) compared with the KITTI defaults (50 ms, ≥ 3).
- Extrinsic calibration is fixed during a run; drift is detected
  (`calibration_monitor_node`) but not corrected.

## License

Apache-2.0. Third-party assets are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
