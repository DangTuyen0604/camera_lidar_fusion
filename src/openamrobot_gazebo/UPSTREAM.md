# OpenAMRobot model provenance

The `openamrobot_description` and `openamrobot_gazebo` packages were imported
from <https://github.com/openAMRobot/openamr-platform-sw> on 2026-09-18.

Upstream robot software is MIT licensed. CAD-derived mesh geometry is licensed
under CERN-OHL-P-2.0. See the bundled `LICENSE` and `LICENSING.md` files.

Local integration changes keep only the standalone mobile-base simulation,
provide a dependency-free empty world, expose spawn pose launch arguments, and
bridge Gazebo joint states to ROS 2. Warehouse scenery is intentionally outside
this initial import.
