# KITTI data

The dataset is intentionally excluded from Git. Download the KITTI raw
sequence used by the demo with:

```bash
./tools/download_kitti.sh
```

Expected layout:

```text
data/kitti/2011_09_26/
├── calib_cam_to_cam.txt
├── calib_velo_to_cam.txt
└── 2011_09_26_drive_0005_sync/
    ├── image_02/{data,timestamps.txt}
    └── velodyne_points/{data,timestamps.txt}
```

There must be 154 PNG images and 154 Velodyne `.bin` files.
