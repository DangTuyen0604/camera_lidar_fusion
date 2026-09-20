"""Docking acceptance and unload invariants."""

from contracts import assert_contains, text, yaml_asset


def test_docking_requires_pose_stop_and_collision_acceptance():
    docking = text('warehouse_mission_manager',
                   'warehouse_mission_manager/docking_controller.py')
    assert_contains(docking, '0.08', '5.0', 'collision', 'linear.x')
    missions = yaml_asset('warehouse_mission_manager',
                          'config/missions.yaml')['missions']
    assert len(missions) == 4
    assert all(mission['destination'] for mission in missions)
