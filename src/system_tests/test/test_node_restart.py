"""Nodes must be restartable and avoid process-global mutable state."""

from contracts import assert_contains, text


def test_python_nodes_have_idempotent_shutdown_paths():
    for package, path in (
            ('warehouse_mission_manager',
             'warehouse_mission_manager/mission_manager.py'),
            ('warehouse_simulation', 'warehouse_simulation/scenario_runner.py')):
        source = text(package, path)
        assert_contains(source, 'destroy_node()', 'rclpy.ok()', 'rclpy.shutdown()')
