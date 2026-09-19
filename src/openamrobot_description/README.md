# openamrobot_description

ROS 2 robot description package for the **OpenAMRobot** mobile base — differential-drive platform with four passive caster wheels and a 2D LiDAR sensor.

Contains: URDF/Xacro model, STL meshes, Gazebo plugins (in xacro), and RViz visualization launch.

## Contents

```
openamrobot_description/
├── launch/
│   └── launch.py               ← RViz + robot_state_publisher (no simulator)
├── meshes/
│   ├── collision/              ← simplified STL meshes for physics
│   └── visual/                 ← full-detail STL meshes for rendering
├── urdf/
│   ├── robo_urdf.urdf.xacro    ← main robot model (includes camera_link + camera_optical_frame)
│   ├── gazebo_control.xacro    ← Gazebo plugins (diff-drive, LiDAR, RGB camera) + friction
│   └── robot.sdf
├── package.xml
└── setup.py
```

## Usage

Visualize the robot in RViz with interactive joint sliders:

```bash
ros2 launch openamrobot_description launch.py
```

For Gazebo simulation, see the `openamrobot_gazebo` package.

## Robot Model

| Link | Collision Geometry |
|---|---|
| `base_footprint` | Massless ground-projected kinematic root |
| `base_link` (4.175 kg) | STL mesh, fixed 0.0635 m above `base_footprint` |
| `left_wheel_link` / `right_wheel_link` (1.061 kg) | Cylinder r=0.11 m |
| `fl/fr/bl/br_caster_link` (0.022 kg) | STL mesh |
| `fl/fr/bl/br_wheel_link` (0.024–0.027 kg) | Sphere r=0.026 m |
| `lidar_link` (0.168 kg) | STL mesh |
| `camera_link` (0.05 kg) | Box 0.04×0.08×0.04 m |
| `camera_optical_frame` | — (optical convention frame) |

**Sensor — LiDAR:** 2D GPU LiDAR — 360°, 10 m max range, 10 Hz, Gaussian noise σ=0.001 m.

**Sensor — Camera:** Forward-facing RGB camera mounted at `xyz="0.35 0 0.20"` on `base_link` — 640×480, 30 Hz, 60° HFOV, 0.05–15 m range, topic `/rgb_image`, Gaussian noise σ=0.005. Frame `camera_optical_frame` follows ROS optical convention.
