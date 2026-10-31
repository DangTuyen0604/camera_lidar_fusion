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

`run_canonical_demo.py` runs one scenario: localize, send a Nav2 goal, insert the physical
worker after the robot starts moving, observe camera/LiDAR detection and fused
XYZ, react through Collision Monitor, expire the obstacle, then reach the goal.
The runner starts a clean MuJoCo/Nav2 graph for every repetition.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 benchmarks/run_canonical_demo.py --runs 10 --keep-going
python3 benchmarks/run_canonical_demo.py --runs 10 --keep-going \
  --detector yolo --model-path models/best.onnx \
  --output benchmarks/canonical_demo_results_yolo.json \
  --csv benchmarks/canonical_demo_results_yolo.csv
```

`--detector` picks the 2D detector (`color` by default, or the `yolo` ONNX
model); `--keep-going` records a failed run instead of stopping. Besides the
first reaction of any kind, each run records when the worker itself was first
detected and fused (`person_detection_latency_ms`, `person_fusion_latency_ms`)
and whether the Collision Monitor reacted before that
(`reacted_before_person_fused`), since it also takes the planar scan.

Per-run measurements and their mean are stored in
`benchmarks/canonical_demo_results.json` and `.csv` (`color`) and
`canonical_demo_results_yolo.json` and `.csv` (`yolo`). The five implemented core
faults are measured with deterministic seed `12014` by:

```bash
python3 benchmarks/run_core_faults.py
```

Its observable results are stored in `experiments/core_faults.json`.
