# Test data

This directory contains small deterministic fixtures that are safe to keep in
Git:

- `calibration/valid_intrinsics.yaml` and `valid_extrinsics.yaml` exercise the
  C++ calibration loader;
- `calibration/invalid_calibration.yaml`, `invalid_kitti_calibration.txt` and
  `wrong_convention.yaml` cover malformed input;
- `calibration/kitti_calibration.txt` is a KITTI calib file.

These fixtures verify parsing only. They are not an
accuracy or performance dataset.
