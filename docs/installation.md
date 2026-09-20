# Cài đặt

Yêu cầu Ubuntu 24.04, ROS 2 Jazzy và ít nhất 8 GB RAM. Cài tự động:

```bash
git clone https://github.com/DangTuyen0604/camera_lidar_fusion.git
cd camera_lidar_fusion
./scripts/install_dependencies.sh
source .venv/bin/activate
./scripts/build_workspace.sh
source install/setup.bash
./scripts/run_tests.sh
```

Script dùng rosdep cho dependency ROS và virtual environment `.venv` cho ONNX,
Ultralytics, benchmark và test Python. Không cài OpenCV pip vào Python hệ thống
vì `cv_bridge` phụ thuộc OpenCV do Ubuntu/ROS cung cấp.

Kiểm tra nhanh sau cài đặt bằng `colcon list` (phải có 12 package) và
`ros2 pkg executables perception_core`.
