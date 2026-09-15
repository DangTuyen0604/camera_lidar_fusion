# Toán học projection

Điểm LiDAR thuần nhất `X_l=[x,y,z,1]^T` được đổi sang camera rectified:

```text
X_c = R_rect * T_camera_lidar * X_l
p   = P * X_c
u   = p_x / p_z,  v = p_y / p_z
```

Chỉ giữ điểm có depth `X_c.z > 0` và pixel nằm trong ảnh. `P` là ma trận 3×4,
`R_rect` và transform là 4×4.

TF2 lưu pose của child trong parent. Nếu YAML mô tả
`camera_optical_frame` trong `velodyne`, projection cần nghịch đảo tương ứng để
đổi điểm từ LiDAR sang camera. Nhầm chiều transform thường tạo overlay phản
chiếu hoặc toàn bộ điểm nằm ngoài ảnh.
