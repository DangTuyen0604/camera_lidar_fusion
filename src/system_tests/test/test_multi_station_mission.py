"""M01-M04 station continuity and cargo lifecycle contract."""

from contracts import assert_contains, text, yaml_asset


def test_four_missions_are_contiguous_and_end_unloaded():
    missions = yaml_asset('warehouse_mission_manager',
                          'config/missions.yaml')['missions']
    assert [item['id'] for item in missions] == ['M01', 'M02', 'M03', 'M04']
    for current, following in zip(missions, missions[1:]):
        assert current['destination'] == following['pickup']
    manager = text('warehouse_mission_manager',
                   'warehouse_mission_manager/mission_manager.py')
    assert_contains(manager, 'MissionState.UNLOADING', 'self.cargo.unload()',
                    'MissionState.COMPLETED')
