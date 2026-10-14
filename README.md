# Camera–LiDAR Fusion for AMR Person Safety

[![Build and test](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/build-and-test.yml)
[![Lint](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/lint.yml/badge.svg)](https://github.com/DangTuyen0604/camera_lidar_fusion/actions/workflows/lint.yml)

A warehouse robot drives toward a goal. A worker steps into the aisle. The
camera finds the worker, the LiDAR measures how far away they are, the fused
3D position becomes an obstacle in the Nav2 costmap, and the Collision Monitor
stops the robot. When the worker leaves, the obstacle expires and the robot
finishes its goal.

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

Canonical scenario (`benchmarks/run_canonical_demo.py`, seed 12012, 3 runs):
the robot is given a Nav2 goal, a worker is spawned in its path only after it
starts moving, then removed once the robot has reacted.

| Metric | Mean | Range over 3 runs |
|---|---:|---:|
| Runs that reached the goal without collision | 3/3 | |
| Worker spawned → Collision Monitor slows/stops the robot | 129 ms | 123–137 ms |
| Sensor timestamp → fused detection published | 87 ms | 82–90 ms |
| Camera–LiDAR timestamp offset | 21 ms | 20.9–21.1 ms |
| Worker removed → obstacle cleared from the costmap | 0.87 s | 0.82–0.95 s |
| LiDAR points on the worker | | 38–39 |
| Camera / point cloud / fusion rate | 9.0 / 7.8 / 4.5 Hz | |

Raw per-run data: [`benchmarks/canonical_demo_results.json`](benchmarks/canonical_demo_results.json).

Two supporting results:

- **Obstacles below the scan plane.** A 0.18 m pallet is invisible to the
  0.46 m-high planar LiDAR. Adding a simulated 16-ring LiDAR with C++ ground
  removal and a 3D costmap layer took full warehouse missions from robot–pallet
  contact in 2/4 runs to 0/3 ([docs/pointcloud_preprocess.md](docs/pointcloud_preprocess.md)).
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
| 6. Nav2 | The obstacle cloud feeds both costmaps and the Collision Monitor. Commands flow `controller_server → velocity_smoother → collision_monitor → /cmd_vel`, so the Collision Monitor has the last word. | [`nav2_stage3_params.yaml`](src/navigation_bringup/config/nav2_stage3_params.yaml) |

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

# Reproduce the results table (headless, 3 runs)
python3 benchmarks/run_canonical_demo.py --runs 3
```

### Any other Linux host (Docker)

```bash
scripts/run_docker_gui.sh ros2 launch fusion_bringup final_demo.launch.py mission:=false
```

Add `--gpu` for NVIDIA rendering. Details in [docs/docker.md](docs/docker.md).

### Other entry points

| Command | What it shows |
|---|---|
| `ros2 launch fusion_bringup final_demo.launch.py` | The full warehouse: four-station pick-and-place missions (M01–M04) with docking, the worker scenario, a pallet that blocks an aisle, and live fusion |
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
| `kitti_ros2_player` | KITTI playback and calibration |
| `system_tests` | Launch-level tests |

`benchmarks/` holds the scenario probes and results, `training/` the YOLO
training scripts, `tools/` data and calibration utilities.

## Limitations

- **Simulation only.** No real robot or real sensor recording has been run
  through the full loop.
- **The simulation detector is color-based.** It finds the worker by color so
  runs are deterministic; YOLOv8 is used only on KITTI and is the
  COCO-pretrained model, not one trained for the warehouse.
- **Fusion accuracy is not yet measured.** The numbers above are rates and
  latencies; the fused position has not been compared against ground truth
  (e.g. KITTI 3D labels).
- **No tracking.** Each frame is fused independently and obstacles carry no
  velocity, so the robot reacts to where a person is, not where they are going.
- **Small sample.** The canonical results are 3 runs of one scenario, with the
  worker placed at a fixed spot.
- The simulation runs fusion with relaxed settings (120 ms sync tolerance,
  ≥ 1 LiDAR point per box) compared with the KITTI defaults (50 ms, ≥ 3).
- The LiDAR mount is partly blocked by the chassis, so navigation masks the
  rear 80° and uses 280° of the scan.
- Extrinsic calibration is fixed during a run; drift is detected
  (`calibration_monitor_node`) but not corrected.

## License

Apache-2.0. Third-party assets are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
