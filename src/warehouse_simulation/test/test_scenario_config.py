from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def test_scenario_supports_all_required_actions():
    root = Path(__file__).parents[1]
    actions = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())
    kinds = {item['action'] for item in actions['scenarios']['warehouse_demo']}
    assert {'spawn', 'delete', 'follow_path', 'attach_cargo', 'detach_cargo',
            'wait_for_robot_region', 'wait_for_mission_state'} <= kinds


def test_two_workers_spawn_immediately_and_patrol_continuously():
    root = Path(__file__).parents[1]
    actions = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())[
        'scenarios']['warehouse_demo']
    worker_spawns = [
        (index, action) for index, action in enumerate(actions)
        if action.get('model') == 'worker']
    first_wait = next(
        index for index, action in enumerate(actions)
        if action.get('action') == 'wait_for_robot_region')

    assert [action['name'] for _, action in worker_spawns] == [
        'crossing_worker', 'station_worker']
    assert [index for index, _ in worker_spawns] == [0, 2]
    for spawn_index, spawn in worker_spawns:
        motion_index = next(
            index for index, action in enumerate(actions)
            if action.get('action') == 'follow_path'
            and action.get('name') == spawn['name'])
        assert motion_index == spawn_index + 1
        assert motion_index < first_wait
        assert actions[motion_index]['loop'] is True
        assert actions[motion_index]['path'][0] == actions[motion_index]['path'][-1]

    deleted = {
        action['name'] for action in actions if action['action'] == 'delete'}
    assert {'crossing_worker', 'station_worker'}.isdisjoint(deleted)


def test_workers_cross_both_station_transfer_routes():
    root = Path(__file__).parents[1]
    actions = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())[
        'scenarios']['warehouse_demo']
    patrols = {
        action['name']: action['path'] for action in actions
        if action.get('action') == 'follow_path'}

    # M01 travels between S1(-6, 3.5) and S2(-2, 3.5); M03 travels between
    # S3(2, 3.5) and S4(6, 3.5).  Each worker must move from below to above
    # y=3.5 at an x coordinate inside the corresponding route segment.
    for name, route_x in (('crossing_worker', -4.0),
                          ('station_worker', 4.0)):
        points = patrols[name]
        assert min(y for _, y in points) <= 2.5
        assert max(y for _, y in points) >= 4.6
        assert min(abs(x - route_x) for x, _ in points) <= 0.21


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
    assert 'ceiling_fixtures' not in models
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
