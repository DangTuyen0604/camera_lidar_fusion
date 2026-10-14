# Calibration monitoring

`calibration_monitor_node` compares the extrinsic in use with a reference
extrinsic and publishes `/fusion/calibration_status`. Default thresholds:

- reprojection error at most 3 px;
- translation drift at most 0.10 m;
- rotation drift at most 1 degree.

The node rereads the file periodically, so `tools/perturb_extrinsic.py` can be
used to test the alarm. On a real system `valid=false` must be treated as a
reason to stop or slow down.

Reprojection error needs a verified set of LiDAR-pixel correspondences. Without
them the monitor still checks geometric drift but reports a projection error of
0; it does not replace calibrating on site.
