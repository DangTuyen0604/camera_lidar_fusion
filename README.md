# Camera–LiDAR Fusion với KITTI và ROS 2

Demo phát ảnh màu, `CameraInfo` và point cloud Velodyne từ KITTI, đồng bộ
timestamp, đọc calibration, chiếu LiDAR lên ảnh, hiển thị bằng RViz và ghi
rosbag.

## Nền tảng đã chọn

Dự án dùng **Ubuntu 24.04 + ROS 2 Jazzy**. Đây là tổ hợp được hỗ trợ trực tiếp
trên máy hiện tại. ROS 2 Humble nhắm tới Ubuntu 22.04 và không được dùng trong
workspace này.

## Cấu trúc chính

- `kitti_ros2_player` (Python): đọc KITTI, tạo message, calibration, projection
  và static TF.
- `perception_core` (C++): calibration, projection, depth estimator,
  `sensor_sync_node` và typed `object_fusion_node`.
- `fusion_bringup`: launch file và cấu hình RViz.
- `fusion_interfaces`: message ROS 2 dùng chung cho detection, calibration,
  synchronization và metrics.
- `yolo_detector`: detector ONNX publish `Detection2DArray`.
- `navigation_bringup` và `navigation_bridge`: tài nguyên Nav2 và điểm mở rộng
  để chuyển fused detection thành vật cản.
- `system_tests`: khung integration test cấp hệ thống.

## Cài đặt

```bash
cd camera_lidar_fusion
./scripts/install_dependencies.sh
```

Nếu ROS 2 Jazzy đã được cài, có thể chỉ cài dependency từ package:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

## Chuẩn bị KITTI

Tải calibration và raw sequence `2011_09_26_drive_0005_sync`:

```bash
./tools/download_kitti.sh
```

Dữ liệu nằm trong `data/kitti/2011_09_26`. Demo yêu cầu 154 ảnh
`image_02` và 154 file `velodyne_points` có cùng frame ID.

## Build và test

```bash
./scripts/build_workspace.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Integration test dùng frame KITTI thật nếu dataset có mặt; nếu chưa tải, test
đó được đánh dấu skip.

## Chạy projection demo

Từ thư mục gốc repository:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fusion_bringup projection_demo.launch.py
```

Không mở RViz:

```bash
ros2 launch fusion_bringup projection_demo.launch.py use_rviz:=false
```

Dataset ở vị trí khác:

```bash
ros2 launch fusion_bringup projection_demo.launch.py \
  dataset_root:=/absolute/path/to/2011_09_26
```

RViz dùng fixed frame `velodyne`, hiển thị point cloud, cây TF và ảnh
projection đã tô màu theo khoảng cách.

## Chạy perception demo hoàn chỉnh

Đặt model ONNX tại `models/yolov8n-opencv.onnx`, sau đó chạy:

```bash
./scripts/run_perception_demo.sh
```

Dataset và model ở vị trí khác được truyền qua launch argument:

```bash
./scripts/run_perception_demo.sh \
  dataset_root:=/path/to/2011_09_26 \
  model_path:=/path/to/yolov8n-opencv.onnx
```

RViz hiển thị ảnh đồng bộ, projection LiDAR, bbox YOLO, bbox kèm depth/XYZ
và marker 3D. Có thể chạy không giao diện bằng `use_rviz:=false`.

## Topic và frame

| Topic | Type | Frame |
|---|---|---|
| `/kitti/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/kitti/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` |
| `/kitti/velodyne/points` | `sensor_msgs/msg/PointCloud2` | `velodyne` |
| `/kitti/camera/lidar_overlay` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/fusion/synced/image` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/fusion/synced/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` |
| `/fusion/synced/points` | `sensor_msgs/msg/PointCloud2` | `velodyne` |
| `/fusion/sync_status` | `fusion_interfaces/msg/SyncStatus` | `camera_optical_frame` |
| `/detections_2d` | `fusion_interfaces/msg/Detection2DArray` | `camera_optical_frame` |
| `/fusion/detections_3d` | `fusion_interfaces/msg/FusedDetectionArray` | `camera_optical_frame` |
| `/fusion/annotated_image` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| `/fusion/object_markers` | `visualization_msgs/msg/MarkerArray` | `camera_optical_frame` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `velodyne → camera_optical_frame` |

Point cloud có bốn field `float32`: `x`, `y`, `z`, `intensity`.
Các publisher dùng reliable QoS để tránh mất frame khi ghi bag cục bộ.

## Chính sách timestamp

Mặc định `timestamp_policy:=rebase_kitti`:

1. Đọc timestamp nanosecond từ `image_02/timestamps.txt` và
   `velodyne_points/timestamps.txt`.
2. Kiểm tra số lượng, thứ tự tăng dần và độ lệch camera–LiDAR. Node dừng nếu
   độ lệch lớn hơn `max_sensor_time_offset_sec` (mặc định 50 ms).
3. Dùng timeline của camera, rebase frame đầu vào thời gian ROS hiện tại và
   giữ nguyên khoảng cách thời gian giữa các frame KITTI.
4. Gán cùng một header stamp cho Image, CameraInfo, PointCloud2 và overlay của
   cùng frame. Khi loop, timestamp tiếp tục tăng và không quay ngược.

Có thể dùng `timestamp_policy:=ros_now` để gán thời gian phát hiện tại, nhưng
chế độ này không giữ timeline gốc của KITTI.

## Chọn định dạng calibration

Player hỗ trợ hai nguồn calibration nhưng tạo cùng một `CameraInfo`, TF và
ma trận projection:

```bash
# Đọc trực tiếp calib_cam_to_cam.txt và calib_velo_to_cam.txt của KITTI
ros2 launch fusion_bringup projection_demo.launch.py \
  calibration_format:=kitti

# Đọc intrinsic và extrinsic từ YAML
ros2 launch fusion_bringup projection_demo.launch.py \
  calibration_format:=yaml
```

Hai file mặc định của chế độ YAML là:

- `src/fusion_bringup/config/camera_intrinsics.yaml`
- `src/fusion_bringup/config/lidar_camera_extrinsics.yaml`

Có thể thay file khi chạy:

```bash
ros2 launch fusion_bringup projection_demo.launch.py \
  calibration_format:=yaml \
  camera_intrinsics_yaml:=/absolute/path/intrinsics.yaml \
  extrinsics_yaml:=/absolute/path/extrinsics.yaml
```

YAML extrinsic dùng quy ước TF2 `child_pose_in_parent`. Với parent
`velodyne` và child `camera_optical_frame`, projection sẽ tự lấy nghịch đảo
để biến điểm LiDAR sang hệ camera. Node từ chối YAML có frame hoặc quy ước sai.

Static TF được tính từ `R_rect_00 × Tr_velo_to_cam`. Vì TF lưu pose của child
trong parent, node phát ma trận nghịch đảo với parent `velodyne` và child
`camera_optical_frame`.

## Ghi và phát rosbag

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fusion_bringup projection_demo.launch.py use_rviz:=false
```

Terminal 2:

```bash
./scripts/record_personal_bag.sh
```

Nhấn `Ctrl+C` để rosbag ghi metadata và đóng file sạch. Có thể truyền đường
dẫn output riêng làm đối số đầu tiên.

```bash
./scripts/record_personal_bag.sh bags/my_demo
ros2 bag info bags/my_demo
ros2 bag play bags/my_demo
```

Bag chứa ảnh gốc, CameraInfo, point cloud, overlay và `/tf_static`.

## Kiểm tra khi chạy

`sensor_sync_node` đồng bộ Image, CameraInfo và PointCloud2 trong tolerance cấu
hình, publish ba topic `/fusion/synced/*` cùng `SyncStatus`. `object_fusion_node`
lookup TF tại timestamp của detection, từ chối cloud sai frame và vẫn publish
detection với `valid=false` khi không ước lượng được depth. Một số lệnh kiểm tra:

```bash
ros2 topic list -t
ros2 topic hz /kitti/velodyne/points
ros2 topic echo /fusion/sync_status --once
ros2 topic echo /fusion/detections_3d --once
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo velodyne camera_optical_frame
```

Overlay và ảnh debug YOLO chỉ được tính khi có subscriber để tránh tốn CPU.
