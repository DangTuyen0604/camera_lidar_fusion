# Chuẩn bị dataset

KITTI raw sequence mặc định là `2011_09_26_drive_0005_sync`. Dùng:

```bash
./tools/download_kitti.sh
python3 tools/prepare_kitti_sequence.py \
  data/kitti/2011_09_26/2011_09_26_drive_0005_sync \
  data/prepared/kitti_0005 --limit 50
```

Mỗi frame cần một ảnh PNG trong `image_02/data`, một point cloud `.bin` trong
`velodyne_points/data`, và hai file timestamp có cùng số dòng. Point cloud
KITTI là dãy little-endian `float32` theo thứ tự `x, y, z, intensity`.

Không commit dataset vào repository. Trước khi chạy demo, kiểm tra frame ID,
số lượng file, timestamp tăng dần và kích thước ảnh khớp calibration.
