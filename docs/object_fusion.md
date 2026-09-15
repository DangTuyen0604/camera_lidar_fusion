# Object fusion

Fusion nhận bbox 2D, CameraInfo và point cloud đã đồng bộ. Point LiDAR được đổi
sang camera frame bằng TF tại đúng timestamp, chiếu qua ma trận `P`, rồi chọn
các điểm nằm trong phần giữa bbox.

Các depth được sắp tăng dần và tách thành cụm theo khoảng trống. Cụm hợp lệ gần
nhất được lọc outlier bằng median absolute deviation; vị trí cuối là median
theo từng trục. Điều này giảm ảnh hưởng của background phía sau bbox.

Output luôn giữ detection gốc. Khi thiếu TF, sai frame, lệch timestamp hoặc
không đủ điểm, `valid=false` và `lidar_point_count=0`. Consumer không được hiểu
tọa độ mặc định `(0, 0, 0)` là một vật thể thật.
