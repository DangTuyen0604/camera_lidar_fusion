"""Box obstacle injection and avoidance contract."""

from contracts import yaml_asset


def test_box_is_spawned_as_a_fused_navigation_obstacle():
    actions = yaml_asset('warehouse_simulation', 'config/scenarios.yaml')[
        'scenarios']['warehouse_demo']
    boxes = [a for a in actions if a.get('model') in ('box', 'cargo_box')]
    assert boxes
    assert all(len(a['pose']) == 4 for a in boxes)
