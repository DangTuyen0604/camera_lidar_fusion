# OpenAMRobot model provenance

The `openamrobot_description` package was imported from
<https://github.com/openAMRobot/openamr-platform-sw> on 2026-09-18, together
with an `openamrobot_gazebo` package that has since been removed in favour of
the MuJoCo simulator in `warehouse_mujoco`.

The original import did not record its upstream commit SHA. The imported tree
first appears in this repository at local commit
`13c76cce0b8c907e2c578fc281ccf37b7b01342b`; no upstream revision is claimed
until it can be verified from the imported content.

Upstream robot software is MIT licensed. CAD-derived mesh geometry is licensed
under CERN-OHL-P-2.0. See the bundled `LICENSE` and `LICENSING.md` files.

The upstream Gazebo plugin tags (`gazebo_control.xacro`, `gazebo_plugins.xacro`)
are kept unchanged as part of the upstream model; they document the sensor and
drive parameters that `warehouse_mujoco` reproduces, and robot_state_publisher
ignores them.
