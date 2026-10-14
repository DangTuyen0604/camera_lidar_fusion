# Dataset preparation

The default KITTI raw sequence is `2011_09_26_drive_0005_sync`:

```bash
./tools/download_kitti.sh
python3 tools/prepare_kitti_sequence.py \
  data/kitti/2011_09_26/2011_09_26_drive_0005_sync \
  data/prepared/kitti_0005 --limit 50
```

Each frame needs a PNG in `image_02/data`, a `.bin` point cloud in
`velodyne_points/data`, and two timestamp files with the same number of lines.
A KITTI point cloud is a little-endian `float32` array of `x, y, z, intensity`.

Do not commit the dataset. Before running a demo, check the frame IDs, the file
counts, that timestamps increase, and that the image size matches the
calibration.
