"""Person safety scenario and collision monitor contract."""

from contracts import assert_contains, text, yaml_asset


def test_worker_crosses_aisle_and_collision_monitor_is_enabled():
    actions = yaml_asset('warehouse_simulation', 'config/scenarios.yaml')[
        'scenarios']['warehouse_demo']
    workers = [a for a in actions if a.get('model') == 'worker']
    assert workers
    assert any(a['action'] == 'follow_path' and a['name'] == workers[0]['name']
               for a in actions)
    nav = text('navigation_bringup', 'config/nav2_params.yaml')
    assert_contains(nav, 'collision_monitor', 'Stop', 'cmd_vel')
