# LiDAR point-cloud preprocessing (C++)

`perception_core/pointcloud_preprocess_node` turns a raw LiDAR scan into
obstacle points and object clusters:

```
PointCloud2 (any layout) -> TF to target_frame -> crop + voxel
  -> RANSAC ground removal -> Euclidean clustering
  -> /lidar/obstacles, /lidar/ground (PointCloud2), /lidar/clusters (MarkerArray)
```

The algorithms are ROS-free classes in `perception_core` (unit-tested with
gtest); the node is a thin composable wrapper.

| Stage | Class | Notes |
|---|---|---|
| Read | `readPointCloud` (`perception_core_ros`) | Any numeric `PointField` type for x/y/z/intensity, either byte order, `row_step` padding; non-finite points skipped and counted |
| Crop + voxel | `PointCloudFilter` | Keeps the highest-intensity point per voxel, in first-seen order |
| Ground | `RansacGroundSegmenter` | Samples only points near the expected floor height, rejects planes that are too steep or at the wrong height, least-squares refit on inliers, fixed seed |
| Clusters | `EuclideanClusterer` | Region growing on a uniform grid (cell = tolerance) with hashed cell lookup |

## Sensor profiles

Everything sensor-specific is a parameter in
`perception_core/config/<profile>.yaml` (installed with the node):

| Profile | Sensor | Frame |
|---|---|---|
| `kitti_hdl64` | KITTI Velodyne HDL-64E, 1.73 m above the road | sensor frame (no base frame in the player) |
| `sim_planar` | MuJoCo warehouse robot, single-plane 360-beam LiDAR | `base_footprint`; ground removal off |
| `sim_vlp16` | MuJoCo warehouse robot, simulated 16-ring LiDAR (`/lidar/points_3d`) | processed in `base_footprint`, published in `lidar_link` (`output_frame: sensor`) |
| `custom_lidar` | Commented template for your own sensor | `base_footprint` |

```bash
ros2 launch fusion_bringup pointcloud_preprocess.launch.py sensor_profile:=kitti_hdl64
ros2 launch fusion_bringup pointcloud_preprocess.launch.py \
  params_file:=/path/to/my_lidar.yaml use_composition:=true
```

`use_composition:=true` loads the node into a component container with
intra-process communication; its publishers hand over `unique_ptr` messages,
so composed subscribers receive them without a copy.

### Using your own LiDAR

1. Copy `custom_lidar.yaml`, set `input_topic` to your driver's topic.
2. Make sure TF connects the cloud's `frame_id` to `target_frame`
   (`base_footprint`). Processing in a floor-level frame makes the ground
   band independent of mounting height and tilt.
3. Tune `crop.*` to your robot, `voxel_size` to the sensor density and
   `clustering.tolerance` to the smallest gap between objects that must stay
   separate.
4. `test_pointcloud_preprocess_profiles.py` rejects keys the node does not
   declare (ROS would silently ignore a typo).

## Navigation integration

The warehouse simulator also models a 16-ring LiDAR on the same mount
(`lidar_3d_channels`, default 16 in `warehouse.launch.py`; +-15 deg, 900
azimuth steps, ~5.6 ms of ray casting per scan). It publishes
`/lidar/points_3d` with x/y/z/intensity float32 and a uint16 `ring` field,
like a Velodyne driver. The planar `/scan` is unchanged and still feeds AMCL.

`navigation.launch.py` runs the node with `sim_vlp16`, and both costmaps
have a `lidar_layer` (STVL) fed by `/lidar/obstacles`:

- marking: points 0.05-2.0 m above the floor, i.e. obstacles under the
  0.46 m scan plane such as the 0.18 m blocking pallet, which previously only
  the camera detection layer could see;
- clearing: a 3D-LiDAR frustum (+-15 deg). Voxels the sensor currently sees
  but no longer hits decay fast; voxels under the lowest beam (a pallet top
  closer than ~1.1 m) persist for `voxel_decay` (15 s) instead of vanishing
  as the robot approaches;
- near-range filter: `sim_vlp16` drops points within 0.40 m horizontally
  (`crop.min_radius`, the 2D scan's `range_min`). Docked at HOME the station
  face is 0.35 m from the centre; the planar scan never saw it, but oblique
  3D returns with 1 cm noise landed in the cell under the footprint edge and
  the robot could not leave the dock (2/2 runs failed their first pickup);
- footprint clearing on, for the same reason.

Full M01-M04 missions (GPU rendering, ground truth from the simulator):

| | before (4 runs, planar LiDAR only) | after (3 runs, 16-ring layer) |
|---|---|---|
| missions completed | 4/4 | 3/3 |
| navigation failures | 0 | 0 |
| docks validated first try | 16/16 | 12/12 |
| mission time | 346-403 s | 349-350 s |
| robot touched the blocking pallet | **2/4 runs** (7.5-8.3 s wheel slip) | **0/3**, min clearance 0.66-1.70 m |

The node processes the ~14k-point simulated scans in about 4 ms per frame.

## Benchmark

`pointcloud_preprocess_benchmark <profile.yaml> [--frames N] [--no-reuse]
[kitti.bin ...]` runs the same pipeline with the same profile, offline.
Without `.bin` files it ray-casts a synthetic HDL-64E-like scene (64 rings x
2083 azimuth steps, road, buildings, cars, poles; 131,856 points/frame), so
numbers are reproducible without downloading KITTI. They are **not** KITTI
results.

`kitti_hdl64` profile, 30 frames, single thread, Intel Core i7-9850H:

| Build / change | filter | ground | cluster | total |
|---|---|---|---|---|
| colcon default build type (`-O0`) | 304 ms | 664 ms | 677 ms | **1645 ms** |
| `-O2`, first implementation | 28.3 | 4.3 | 29.2 | **61.8 ms** |
| splitmix64 voxel/cell hash | 8.1 | 4.7 | 27.0 | 39.7 ms |
| visited points swapped out of their cell | 8.2 | 5.0 | 19.7 | 32.8 ms |
| neighbour cells cached per cell | 7.5 | 4.7 | 9.2 | **21.4 ms** |

- colcon passes no build type, so every package was compiled at `-O0`: 16x
  over a 10 Hz LiDAR's 100 ms budget. `perception_core` now defaults to
  `RelWithDebInfo`.
- `std::hash<int64_t>` is the identity in libstdc++; the old voxel key hash
  (`hx ^ hy << 1 ^ hz << 2`) put neighbouring voxels in neighbouring buckets.
- On dense surfaces every frontier point rescanned points already in the
  cluster; each cell now keeps its unvisited points in front.
- The 27 neighbour cells of a cell were looked up again for every point in
  it; they are now resolved once per cell.
- Reusing buffers across frames (`--no-reuse` to compare) changed the total
  by under 1 ms: allocation was never the bottleneck.

Clustering results are identical across all rows (24,991 obstacle points,
32.2 clusters per frame).

## Limitations

- The simulated 16-ring LiDAR is ideal: 1 cm Gaussian range noise, no
  motion distortion, dropouts or reflectivity effects.
- Leaving the HOME dock is slow and sometimes needs Spin/BackUp recoveries:
  docked, the robot centre is 3 cm from the dock's inscribed-inflation zone.
  Runs with the 16-ring layer had 3-5 such recoveries (0-1 before), but an
  A/B test with the layer disabled at runtime showed the same, so it is not
  attributed to the layer. Not fixed.
- The voxel map and each cluster's index list are still allocated per frame.
- Clusters are axis-aligned boxes, without tracking or classification.
