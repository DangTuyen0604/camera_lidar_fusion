# Benchmarks

## Canonical person-safety scenario

```bash
python3 benchmarks/run_canonical_demo.py --runs 3
```

Each run launches `final_demo.launch.py mission:=false` headless, sends a Nav2
goal, spawns a worker in the robot's path once it moves, waits for detection,
fusion, a bridge obstacle and a Collision Monitor reaction, removes the worker,
checks that the obstacle expires, and waits for the goal to succeed. The
results are `benchmarks/canonical_demo_results.json` and `.csv`; the README
table is taken from them.

## Fault injection

```bash
python3 benchmarks/run_core_faults.py --seed 12014
```

Measures the effect of a timestamp delay, an extrinsic perturbation,
point-cloud noise, density reduction and a lost detection. Results:
`experiments/gate14_core_faults.json`.

## Scenario matrix

`benchmarks/scenarios.yaml` defines 16 configurations (timestamp delay,
extrinsic drift, point-cloud noise and density, detector confidence, fusion
timeout, slow camera or LiDAR). Each run starts a fresh process group, waits
for real ROS metrics, samples CPU/RSS from `/proc`, and cleans up with bounded
SIGINT/SIGTERM/SIGKILL. Missing metrics fail the run; they are never replaced
by constants or example data.

```bash
python3 benchmarks/run_benchmark.py --dry-run
python3 benchmarks/run_benchmark.py --runs 5
python3 benchmarks/plot_results.py
```

Outputs go to `benchmark_results/` (`raw_results.csv`, `summary.csv`, plots).
No results of this matrix are checked in yet. Compare runs only on the same
machine, model, dataset and ROS configuration.
