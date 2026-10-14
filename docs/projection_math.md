# Projection math

A homogeneous LiDAR point `X_l = [x, y, z, 1]^T` is moved into the rectified
camera frame and projected:

```text
X_c = R_rect * T_camera_lidar * X_l
p   = P * X_c
u   = p_x / p_z,  v = p_y / p_z
```

Only points with depth `X_c.z > 0` whose pixel lies inside the image are kept.
`P` is 3x4; `R_rect` and the transform are 4x4.

TF2 stores the pose of the child frame in its parent. If the YAML describes
`camera_optical_frame` in `velodyne`, the projection needs the inverse of that
transform to move points from the LiDAR into the camera. Getting the direction
wrong typically gives a mirrored overlay, or puts every point outside the image.
