"""Dynamic pallet insertion must reach the Nav2 replanning path."""

from contracts import yaml_asset


def test_pallet_event_is_position_triggered_and_removed_after_replan_window():
    actions = yaml_asset('warehouse_simulation', 'config/scenarios.yaml')[
        'scenarios']['warehouse_demo']
    pallet = [a for a in actions if a.get('model') == 'pallet']
    assert pallet and pallet[0]['action'] == 'spawn'
    assert any(a['action'] == 'delete' and a['name'] == pallet[0]['name']
               for a in actions)
    assert any(a['action'] == 'wait_for_robot_region' for a in actions)
