# Demo map

`demo_map.pgm` is a deterministic 5 m × 5 m map used for launch validation and
local navigation demonstrations. Black pixels are occupied and light pixels
are free. Replace both map files with an environment map produced by
`slam_toolbox` before driving a real robot.
# Mapping and reload workflow

Start the warehouse and `mapping.launch.py`, teleoperate through every aisle,
then save the actual SLAM map with:

```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: warehouse_map, image_format: pgm, map_mode: trinary}"
```

Pass the resulting YAML to localization or the full demo with
`map:=/absolute/path/warehouse_map.yaml`. AMCL owns `map -> odom`; Gazebo owns
only `odom -> base_footprint`, so the TF tree has no duplicate publisher.
