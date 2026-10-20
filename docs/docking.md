# Docking and cargo hand-off

Navigation first targets a staging pose 0.70 m before the station. The docking
action then performs final alignment. A dock is accepted only when position
error is below 0.08 m, yaw error below 5 degrees, linear/angular speed below
0.02, and no collision is active.

Leaving a dock is always `DockRobot` then `UndockRobot` (backing out to the
staging pose). `UndockRobot` refuses ("Robot is not in the dock") unless the
dock plugin knows the dock's pose and sees the robot within its 5 cm
`docking_threshold`, which is stricter than the 8 cm contract above: at boot
the plugin has no pose at all, and a dock once settled at exactly 5.0 cm. The
extra `DockRobot` returns at once when the robot is within 5 cm and re-seats
it otherwise. After three failed undocks the mission lets navigation leave
the station instead of retrying forever.

The robot boots docked at HOME, facing the station like every dock pose, and
the first mission starts by undocking. Spawned sideways against the dock
instead, the station was 3 cm from the body, closer than one costmap cell:
the controller, Spin and BackUp all reported a collision and the robot spent
35-40 s in recoveries before it got away.

Cargo unload occurs only after `DOCKED`; `COMPLETED` is published after the
unload delay. Benchmark collection computes docking error from live odometry
against the configured station coordinates.
