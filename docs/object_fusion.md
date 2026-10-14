# Object fusion

Fusion takes the 2D boxes, CameraInfo and the synchronized point cloud. LiDAR
points are moved into the camera frame with TF at the detection timestamp,
projected through `P`, and the points inside the central part of each box are
selected.

Their depths are sorted and split into clusters at depth gaps. The nearest
cluster with enough points is filtered for outliers with the median absolute
deviation; the final position is the per-axis median. This limits the effect
of background points that fall inside the box.

The output always keeps the original detection. When TF is missing, the frame
is wrong, the timestamps are too far apart or there are too few points, the
result has `valid=false` and `lidar_point_count=0`. Consumers must not read
the default `(0, 0, 0)` position as a real object.
