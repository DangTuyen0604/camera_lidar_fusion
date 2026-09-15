# Đồng bộ timestamp

`sensor_sync_node` dùng ApproximateTime cho Image, CameraInfo và PointCloud2.
Mặc định queue là 20 và spread tối đa 50 ms. Bộ ba hợp lệ được phát nguyên
header sang `/fusion/synced/*`; trạng thái nằm ở `/fusion/sync_status`.

Với KITTI, `rebase_kitti` giữ khoảng thời gian tương đối nhưng đặt frame đầu ở
thời gian ROS hiện tại. `ros_now` phù hợp demo đơn giản nhưng làm mất timing
gốc. Timestamp không được quay ngược khi lặp dataset.

Theo dõi `camera_lidar_offset_ms`, `synchronized_pairs` và `dropped_messages`.
Nếu offset có bias cố định, sửa đồng bộ clock hoặc calibration thời gian thay vì
nới tolerance vô hạn.
