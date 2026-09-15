# Giám sát calibration

`calibration_monitor_node` so sánh extrinsic đang dùng với extrinsic tham chiếu
và publish `/fusion/calibration_status`. Ba ngưỡng mặc định:

- sai số reprojection tối đa 3 px;
- drift tịnh tiến tối đa 0,10 m;
- drift xoay tối đa 1 độ.

Node đọc lại file theo chu kỳ nên có thể dùng cùng
`tools/perturb_extrinsic.py` để kiểm thử cảnh báo. Trạng thái `valid=false` phải
được coi là điều kiện dừng hoặc giảm tốc đối với hệ thống chạy thật.

Sai số reprojection cần tập correspondence LiDAR–pixel đã xác minh. Khi không
có correspondence, monitor vẫn kiểm tra drift hình học nhưng báo projection
error bằng 0; đây không thay thế quy trình calibration thực địa.
