from pathlib import Path
import xml.etree.ElementTree as ET

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


def test_world_matches_navigation_map_boundary():
    root = Path(__file__).parents[1]
    world = ET.parse(root / 'worlds' / 'warehouse.sdf').getroot()
    models = {model.attrib['name']: model for model in world.findall('.//model')}

    assert 'ground_plane' in models
    assert models['ground_plane'].find('.//plane') is not None
    assert {'wall_west', 'wall_east', 'wall_south', 'wall_north'} <= models.keys()

    poses = {
        name: [float(value) for value in models[name].findtext('pose').split()]
        for name in ('wall_west', 'wall_east', 'wall_south', 'wall_north')
    }
    assert poses['wall_west'][:2] == [-8.8, 0.0]
    assert poses['wall_east'][:2] == [8.8, 0.0]
    assert poses['wall_south'][:2] == [0.0, -6.8]
    assert poses['wall_north'][:2] == [0.0, 6.8]
