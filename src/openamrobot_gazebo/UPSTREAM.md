# OpenAMRobot model provenance

The `openamrobot_description` and `openamrobot_gazebo` packages were imported
from <https://github.com/openAMRobot/openamr-platform-sw> on 2026-09-18.

The original import did not record its upstream commit SHA. The imported tree
first appears in this repository at local commit
`13c76cce0b8c907e2c578fc281ccf37b7b01342b`; no upstream revision is claimed
until it can be verified from the imported content.

Upstream robot software is MIT licensed. CAD-derived mesh geometry is licensed
under CERN-OHL-P-2.0. See the bundled `LICENSE` and `LICENSING.md` files.

Local integration changes keep only the standalone mobile-base simulation,
provide a dependency-free empty world, expose spawn pose launch arguments, and
bridge Gazebo joint states to ROS 2. Warehouse scenery is intentionally outside
this initial import.
