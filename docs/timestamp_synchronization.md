# Timestamp synchronization

`sensor_sync_node` uses `ApproximateTime` over Image, CameraInfo and
PointCloud2. The defaults are a queue of 20 and a maximum spread of 50 ms. A
valid triple is republished with its original headers on `/fusion/synced/*`;
status is on `/fusion/sync_status`.

For KITTI, `rebase_kitti` keeps the relative timing but puts the first frame at
the current ROS time. `ros_now` suits a simple demo but loses the original
timing. Timestamps never go backwards when the dataset loops.

Watch `camera_lidar_offset_ms`, `synchronized_pairs` and `dropped_messages`. If
the offset has a constant bias, fix the clock synchronization or the time
calibration instead of widening the tolerance.
