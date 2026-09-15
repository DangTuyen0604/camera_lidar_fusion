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
