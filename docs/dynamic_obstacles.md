# Dynamic obstacles and replanning

`detection_obstacle_bridge_node` consumes typed 3D detections, rejects invalid,
low-confidence, non-finite, out-of-range, stale, future-dated and untransformable
samples, then publishes a `PointCloud2` obstacle source in `base_link`.

Both Nav2 costmaps enable marking and clearing for
`/navigation/detection_obstacles`. After `obstacle_timeout` the bridge publishes
clearing rays and an empty cloud so a disappeared pallet cannot become a ghost
obstacle. Diagnostics expose every rejection and clearing counter.

The warehouse scenario spawns a blocking pallet after the robot enters the
aisle. The Day-6 test records `/plan` before and after insertion and requires a
different path without collision.
