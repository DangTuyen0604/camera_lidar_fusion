# Giới hạn

- Pipeline mẫu dùng CPU ONNX Runtime; tốc độ phụ thuộc model và phần cứng.
- KITTI cung cấp dữ liệu đã ghi, không đại diện đầy đủ cho rung, mưa hoặc lệch
  clock của sensor thật.
- Depth theo bbox dùng median của cụm điểm gần nhất; vật thể ít điểm hoặc che
  khuất có thể cho `valid=false`.
- Calibration monitor không tự tối ưu extrinsic và chỉ phát hiện drift theo
  ngưỡng cấu hình.
- Obstacle bridge publish tâm vật thể dạng PointCloud2, chưa mô hình hóa kích
  thước hay vận tốc vật thể.
- Robot URDF và map trong `navigation_bringup` chỉ phục vụ demo cấu trúc; cần
  thay bằng mô hình, footprint và dynamics của robot thật.

Không dùng pipeline này làm cơ chế an toàn duy nhất trên phương tiện thật.
