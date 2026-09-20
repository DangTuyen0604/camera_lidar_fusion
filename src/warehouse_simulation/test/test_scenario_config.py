from pathlib import Path

import yaml


def test_scenario_supports_all_required_actions():
    root = Path(__file__).parents[1]
    actions = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())
    kinds = {item['action'] for item in actions['scenarios']['warehouse_demo']}
    assert {'spawn', 'delete', 'follow_path', 'attach_cargo', 'detach_cargo',
            'wait_for_robot_region', 'wait_for_mission_state'} <= kinds


def test_all_models_are_self_contained():
    root = Path(__file__).parents[1]
    for name in ('shelf', 'pallet', 'cargo_box', 'worker', 'conveyor', 'docking_station'):
        assert (root / 'models' / name / 'model.sdf').is_file()
        assert (root / 'models' / name / 'model.config').is_file()
