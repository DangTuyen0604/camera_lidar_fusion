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
