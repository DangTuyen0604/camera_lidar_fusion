# Day 1 Stage 3 status

## Reproduce

From a clean terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch navigation_bringup stage3.launch.py
```

The launch starts the warehouse simulation, filtered LiDAR, map localization,
Nav2, velocity smoother, collision monitor, and (by default) RViz. Use
`use_rviz:=false` for headless validation.

## Command chain

```text
controller_server / behavior_server
  -> /cmd_vel_nav
velocity_smoother
  -> /cmd_vel_smoothed
collision_monitor
  -> /cmd_vel
ros_gz_bridge
  -> Gazebo DiffDrive
```

Runtime topic introspection found the intended controller/behavior publishers
and a single `velocity_smoother` subscriber on `/cmd_vel_nav`, then exactly one
publisher and one subscriber at both downstream safety boundaries. The final
`/cmd_vel` publisher was only `collision_monitor`, and its only subscriber was
`ros_gz_bridge`.

The velocity smoother was active with simulation time enabled, a 20 Hz
frequency, velocity limits `[0.5, 0.0, 1.0]` / `[-0.2, 0.0, -1.0]`, and
acceleration/deceleration limits `[1.5, 0.0, 2.5]` /
`[-1.5, 0.0, -2.5]`. During motion, all three command topics measured about
20 Hz.

## Runtime evidence

| Check | Result |
| --- | --- |
| Straight goal `(0.8, 0.0)` | `SUCCEEDED`, 7 s |
| Turn goal `(1.5, 1.0, yaw=90 deg)` | `SUCCEEDED`, 11 s |
| Around-shelf goal `(4.0, 4.0)` | `SUCCEEDED`, 21 s |
| Appearing obstacle | slowdown applied; goal subsequently `SUCCEEDED` |

For the appearing-obstacle test, a repository cargo box was spawned 0.80 m
in front of the moving robot. The collision monitor reported `SlowZone`, the
minimum output/input velocity ratio was 0.30, minimum robot-to-obstacle center
clearance was 1.023 m, and normal motion resumed after the obstacle was
removed. A full stop was not needed because the slowdown zone kept the robot
outside the stop zone.

Observed topic rates:

- `/clock`: about 1000 Hz
- `/odom`: about 49.7 Hz
- `/scan_filtered`: about 9.91 Hz
- `/map`: transient-local static publication; loaded by the global costmap

TF remained available for `map -> odom -> base_link -> lidar_link`. The static
`base_link -> lidar_link` translation was approximately
`[0.351, -0.001, 0.168]`. All required navigation lifecycle nodes were active:
`map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`,
`bt_navigator`, `velocity_smoother`, and `collision_monitor`.

Both costmaps were live. The global costmap loaded the saved warehouse map;
the local costmap was rolling and consumed `/scan_filtered`. No persistent TF,
lifecycle, or topic regression was observed. Two transient costmap transform
warnings occurred only during startup.

After the clean build, the same launch command was repeated from a new shell.
All required lifecycle nodes became active without manual setup and a fresh
goal at `(0.8, 0.0)` finished `SUCCEEDED`. Ctrl-C shut down Gazebo, the bridge,
filters, costmaps, navigation nodes, and lifecycle managers without leaving a
background process.
