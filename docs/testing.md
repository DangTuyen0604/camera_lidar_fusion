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
# Day-6 test gates

The `system_tests` package contains independent contracts for topics, TF,
perception, fusion output, navigation bridge, replanning, person stop/resume,
box avoidance, docking, four-station missions, restart, rosbag replay, clean
shutdown and fault injection.

Run Gate 6.1 later from a clean shell:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

No important test may be skipped. After launch tests, verify that no descendant
ROS, Gazebo or launch process remains. The live warehouse end-to-end sequence
must observe Nav2 activation, mission start, pallet replan, worker stop/resume,
box avoidance, docking, unload, mission completion and zero collisions.
