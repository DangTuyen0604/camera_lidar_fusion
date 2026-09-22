# Live full-system benchmarks

`run_benchmark.py` owns the ROS launch process, samples CPU/RSS from `/proc`,
and invokes `metrics_collector.py` against live topics. A run fails when a
metric is absent; no sample/default data is substituted.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 benchmarks/run_benchmark.py
python3 benchmarks/plot_results.py
```

Use `--dry-run` to validate all 16 scenario definitions without launching the
stack. Real CSV and PNG artifacts are written only under `benchmark_results/`.

## Canonical perception-to-navigation demo

Gate 12 uses one scenario only: localize, send a Nav2 goal, insert the physical
worker after the robot starts moving, observe camera/LiDAR detection and fused
XYZ, react through Collision Monitor, expire the obstacle, then reach the goal.
The runner starts a clean Gazebo/Nav2 graph for every repetition.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 benchmarks/run_canonical_demo.py --runs 3
```

Measured per-run evidence and the Gate 13 aggregate are stored in
`benchmarks/canonical_demo_results.json` and `.csv`. The five implemented core
faults are measured with deterministic seed `12014` by:

```bash
python3 benchmarks/run_core_faults.py
```

Its observable results are stored in `experiments/gate14_core_faults.json`.
