# Tích hợp navigation

`detection_obstacle_bridge_node` nhận `/fusion/detections_3d`, lọc detection
không hợp lệ, confidence thấp và vật thể ngoài phạm vi, rồi publish
`/navigation/detection_obstacles` dạng `sensor_msgs/PointCloud2`.

Local/global costmap cấu hình topic này làm marking source. TF phải nối frame
của detection với `map`/`odom`; nếu thiếu TF, Nav2 sẽ bỏ message.

```bash
source install/setup.bash
ros2 launch navigation_bringup full_system.launch.py
ros2 topic echo /navigation/detection_obstacles --once
```

Trên robot thật, thay `demo_map`, URDF, topic odometry và laser scan. Kiểm tra
footprint/inflation radius ở tốc độ thấp trước khi cho phép điều khiển tự động.
