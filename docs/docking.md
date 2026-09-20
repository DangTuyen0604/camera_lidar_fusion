# Docking and cargo hand-off

Navigation first targets a staging pose 0.70 m before the station. The docking
action then performs final alignment. A dock is accepted only when position
error is below 0.08 m, yaw error below 5 degrees, linear/angular speed below
0.02, and no collision is active.

Cargo unload occurs only after `DOCKED`; `COMPLETED` is published after the
unload delay. Benchmark collection computes docking error from live odometry
against the configured station coordinates.
