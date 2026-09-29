# warehouse_mujoco

MuJoCo physics for the OpenAMRobot warehouse, started by
`warehouse_simulation/warehouse.launch.py`. It replaced the former Gazebo
server, `ros_gz_bridge` and robot spawner with the same ROS contract, so Nav2,
the fusion pipeline, the mission manager and the scenario runner are unchanged.

## Install

MuJoCo has no rosdep key on Jazzy; install the wheel for the system Python
that ROS uses:

```bash
pip install --user --break-system-packages mujoco
colcon build --symlink-install --packages-up-to warehouse_simulation
```

## Run

```bash
# Warehouse + scenario
ros2 launch warehouse_simulation warehouse.launch.py gui:=true
# Full stack: every launch that includes warehouse.launch.py uses MuJoCo
ros2 launch navigation_bringup warehouse_full_demo.launch.py
# Headless machines (Docker, CI): render the camera without a display
MUJOCO_GL=egl ros2 launch warehouse_simulation warehouse.launch.py   # or osmesa
# Drive by hand
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ROS contract

| Direction | Topic / service | Type | Frame / rate |
|---|---|---|---|
| sub | `/cmd_vel` | `geometry_msgs/Twist` | limits of the former DiffDrive plugin |
| pub | `/clock` | `rosgraph_msgs/Clock` | 250 Hz |
| pub | `/odom`, `/tf` | `nav_msgs/Odometry`, `odom -> base_footprint` | wheel odometry, 50 Hz |
| pub | `/joint_states` | `sensor_msgs/JointState` | 50 Hz |
| pub | `/imu` | `sensor_msgs/Imu` | `imu_link`, 100 Hz |
| pub | `/scan` | `sensor_msgs/LaserScan` | `lidar_link`, 360 beams, 0.4-10 m, 10 Hz |
| pub | `/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_link`, 10 Hz |
| pub | `/camera/image_raw`, `/camera/camera_info` | `rgb8` 640x480, HFOV 1.047 | `camera_optical_frame`, 12 Hz |
| pub | `/ground_truth/odom` | `nav_msgs/Odometry` | `world -> base_footprint` (extra, for benchmarks) |
| srv | `/mujoco/{spawn_entity,set_entity_state,delete_entity}` | `simulation_interfaces` | used by `scenario_runner` |

## Model notes

* `mjcf/amr.xml` mirrors `robo_urdf.urdf.xacro` (link poses, masses,
  inertias, CAD meshes for visuals). The base mesh exceeds MuJoCo's
  200k-face STL limit and is decimated in memory at load.
* The chassis collides as the Nav2 footprint (cylinder, radius 0.32 m), not as
  the CAD shell. Docks, map and station poses were laid out for that
  footprint; the CAD nose would sit 7 cm inside each dock at its docking pose
  and sweep into the HOME dock when turning in place. Gazebo's lenient mesh
  collision hid this mismatch.
* Friction reproduces Gazebo's pairs although MuJoCo combines geoms with
  max() instead of min(): world geometry uses mu=0.2, wheels 1.0, chassis 0.2.
* The drive wheels have a 25 N spring preload. With six rigid contacts the
  wheel/caster load split is hypersensitive to sub-millimetre geometry and
  the wheels lose traction; with the preload, wheel odometry tracks ground
  truth to about 0.5 %.
* The warehouse is converted from `warehouse_simulation/worlds/warehouse.sdf`
  at start-up, so the SDF remains the single source of the layout.
* MuJoCo cannot add bodies at run time: every spawnable model (`worker`,
  `cargo_box`, `pallet`) gets a pool of hidden mocap bodies
  (`entity_pool_size`, default 4). Cargo posed on top of the robot is
  contact-free so the kinematic body cannot pin the robot.
* The LiDAR ray-casts against rendered world geometry (like `gpu_lidar`) and
  ignores the robot itself.
