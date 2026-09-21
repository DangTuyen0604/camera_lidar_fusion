# Warehouse simulation

The warehouse is a deterministic Gazebo Sim world with four stations, aisles,
shelves, conveyors, pallets, workers and cargo boxes. Start it headless with:

```bash
ros2 launch warehouse_simulation warehouse.launch.py gui:=false use_scenario:=true
```

`scenario_runner` advances only on robot-region or mission-state events. Spawn,
move and delete operations use simulator services and publish matching simulator
truth on `/benchmark/ground_truth/detections_3d`. It also publishes benchmark
events and a live proximity collision counter. Scenario entities are deleted at
the end so repeated runs begin from the same state.

The complete one-command stack is:

```bash
ros2 launch navigation_bringup warehouse_full_demo.launch.py gui:=false
```

For the operator-facing simulation with Gazebo GUI, RViz and mission autostart,
use the top-level bringup:

```bash
ros2 launch fusion_bringup bringup_sim.launch.py
```
