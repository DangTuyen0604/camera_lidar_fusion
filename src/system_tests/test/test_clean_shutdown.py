"""Launch/process shutdown ownership contracts."""

from contracts import assert_contains, text


def test_runtime_nodes_and_launches_have_bounded_shutdown_ownership():
    scenario = text('warehouse_simulation',
                    'warehouse_simulation/scenario_runner.py')
    mission = text('warehouse_mission_manager',
                   'warehouse_mission_manager/mission_manager.py')
    assert_contains(scenario, 'finally:', 'destroy_node()', 'rclpy.shutdown()')
    assert_contains(mission, 'finally:', 'destroy_node()', 'rclpy.shutdown()')
