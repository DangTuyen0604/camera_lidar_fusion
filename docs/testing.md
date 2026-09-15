# Kiểm thử

Gate chuẩn:

```bash
colcon list
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Unit test bao phủ calibration parser, rigid transform, projection, depth
estimation, point filtering, latency và obstacle conversion. System test kiểm
tra message graph, install assets và pipeline perception tối thiểu.

CI chạy lại gate trên Ubuntu 24.04/ROS 2 Jazzy. Khi thất bại, XML trong
`build/**/test_results` được upload làm artifact. Test phụ thuộc KITTI/model
phải tự skip với lý do rõ ràng; test cấu trúc và thuật toán không được skip.
