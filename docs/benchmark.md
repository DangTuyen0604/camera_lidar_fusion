# Benchmark

Chạy benchmark lặp lại một lệnh và ghi CSV:

```bash
python3 benchmarks/run_benchmark.py --runs 10 -- \
  ros2 launch fusion_bringup perception_demo.launch.py use_rviz:=false
python3 benchmarks/plot_results.py
```

Các cột chính là `scenario`, `run`, `latency_ms`, `return_code` và thời điểm
chạy. Chỉ so sánh các lần chạy trên cùng máy, model, dataset và cấu hình ROS.
P50 phản ánh độ trễ thông thường; P95 dùng làm gate để phát hiện regression.

Kết quả phát sinh được lưu trong `benchmarks/results/`. CSV mẫu chỉ mô tả
schema và không được xem là số đo hiệu năng của phần cứng thật.
# Full-system benchmark

The benchmark runs all 16 configurations at least five times. Each run starts a
fresh process group, waits for real ROS metrics, samples CPU/RSS from `/proc`,
then performs bounded SIGINT/SIGTERM/SIGKILL cleanup. Missing metrics fail the
run; they are never replaced by constants or example data.

```bash
python3 benchmarks/run_benchmark.py --dry-run
python3 benchmarks/run_benchmark.py --runs 5
python3 benchmarks/plot_results.py
```

Outputs are `benchmark_results/raw_results.csv`, `summary.csv`, and seven PNG
plots. Gates are zero collisions, 100% person stops, at least 90% mission
success, p95 fusion latency below 150 ms, docking error below 0.08 m, and ghost
obstacle lifetime bounded by the configured timeout plus tolerance.
