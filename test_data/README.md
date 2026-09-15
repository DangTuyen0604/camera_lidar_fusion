# Test data

This directory contains small deterministic fixtures that are safe to keep in
Git:

- `calibration/valid_intrinsics.yaml` and `valid_extrinsics.yaml` exercise the
  C++ calibration loader;
- `calibration/invalid_calibration.yaml` covers malformed input;
- `images/sample.png` is a 320×120 RGBA image for loader/tool smoke tests;
- `pointclouds/sample.bin` contains four KITTI-format points encoded as
  little-endian `float32` records `(x, y, z, intensity)`.

These fixtures verify parsing and message conversion only. They are not an
accuracy or performance dataset.
