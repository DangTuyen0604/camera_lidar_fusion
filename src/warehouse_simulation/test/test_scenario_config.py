from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import math

from warehouse_simulation.pedestrian_roaming import RoamingAgent, WaypointGraph
from warehouse_simulation.scenario_runner import ScenarioRunner
import yaml


def test_scenario_supports_all_required_actions():
    root = Path(__file__).parents[1]
    actions = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())
    kinds = {item['action'] for item in actions['scenarios']['warehouse_demo']}
    assert {'spawn', 'delete', 'roam',
            'wait_for_robot_region', 'wait_for_mission_state'} <= kinds


def test_two_workers_spawn_immediately_and_roam_continuously():
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
            if action.get('action') == 'roam'
            and action.get('name') == spawn['name'])
        assert motion_index == spawn_index + 1
        assert motion_index < first_wait
        assert actions[motion_index]['speed_range'] == [0.4, 0.8]
        assert actions[motion_index]['pause_range'] == [0.5, 2.0]

    deleted = {
        action['name'] for action in actions if action['action'] == 'delete'}
    assert {'crossing_worker', 'station_worker'}.isdisjoint(deleted)


def test_actor_validation_scales_to_any_number_of_workers():
    actions = []
    expected_names = set()
    for index in range(4):
        name = f'worker_{index}'
        expected_names.add(name)
        actions.extend([
            {
                'action': 'spawn',
                'name': name,
                'model': 'worker',
                'pose': [float(index), float(index + 1), 0.0, 0.0],
            },
            {
                'action': 'follow_path',
                'name': name,
                'speed': 0.5,
                'loop': True,
                'path': [[index, index + 1], [index, index + 2]],
            },
        ])

    assert ScenarioRunner._validate_actors(actions) == expected_names


def test_actor_validation_rejects_duplicate_names_and_poses():
    duplicate_name = [
        {'action': 'spawn', 'name': 'worker', 'model': 'worker',
         'pose': [0.0, 0.0, 0.0, 0.0]},
        {'action': 'spawn', 'name': 'worker', 'model': 'worker',
         'pose': [1.0, 0.0, 0.0, 0.0]},
        {'action': 'follow_path', 'name': 'worker', 'path': [[0.0, 0.0]]},
    ]
    try:
        ScenarioRunner._validate_actors(duplicate_name)
    except ValueError as error:
        assert 'names must be unique' in str(error)
    else:
        raise AssertionError('duplicate worker name was accepted')

    duplicate_pose = [
        {'action': 'spawn', 'name': 'worker_a', 'model': 'worker',
         'pose': [0.0, 0.0, 0.0, 0.0]},
        {'action': 'follow_path', 'name': 'worker_a', 'path': [[0.0, 0.0]]},
        {'action': 'spawn', 'name': 'worker_b', 'model': 'worker',
         'pose': [0.0, 0.0, 0.0, 1.0]},
        {'action': 'follow_path', 'name': 'worker_b', 'path': [[0.0, 0.0]]},
    ]
    try:
        ScenarioRunner._validate_actors(duplicate_pose)
    except ValueError as error:
        assert 'different initial poses' in str(error)
    else:
        raise AssertionError('overlapping worker poses were accepted')


def test_completed_motion_response_is_consumed_and_pose_is_committed():
    result = SimpleNamespace(RESULT_OK=1, result=1, error_message='')
    future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(result=result),
    )
    runner = SimpleNamespace(
        motion_futures={'worker_0': (future, 0.0, [1.0, 2.0, 0.0, 0.0])},
        action_timeout_sec=10.0,
        entities={'worker_0': {'pose': [0.0, 0.0, 0.0, 0.0]}},
        _fail_command=lambda *args: None,
    )

    assert ScenarioRunner._finish_motion(runner, 'worker_0')
    assert runner.entities['worker_0']['pose'] == [1.0, 2.0, 0.0, 0.0]
    assert 'worker_0' not in runner.motion_futures


def test_in_flight_motion_does_not_publish_unconfirmed_pose():
    future = SimpleNamespace(done=lambda: False)
    runner = SimpleNamespace(
        motion_futures={},
        pose_client=SimpleNamespace(),
        entities={'worker_0': {'pose': [0.0, 0.0, 0.0, 0.0]}},
        _pose=lambda pose: pose,
        _start=lambda *args: future,
    )

    assert ScenarioRunner._move(
        runner, 'worker_0', [1.0, 2.0, 0.0, 0.0], advance=False)
    assert runner.entities['worker_0']['pose'] == [0.0, 0.0, 0.0, 0.0]
    assert runner.motion_futures['worker_0'][0] is future


def test_scenario_move_carries_pose_until_service_response():
    future = SimpleNamespace(done=lambda: False)
    starts = []
    runner = SimpleNamespace(
        motion_futures={},
        pose_client=SimpleNamespace(),
        entities={'worker_0': {'pose': [0.0, 0.0, 0.0, 0.0]}},
        _pose=lambda pose: pose,
        _start=lambda *args: starts.append(args) or future,
    )
    target = [1.0, 2.0, 0.0, 0.5]

    assert ScenarioRunner._move(runner, 'worker_0', target, advance=True)
    assert runner.entities['worker_0']['pose'] == [0.0, 0.0, 0.0, 0.0]
    assert starts[0][-1] == {
        'operation': 'move', 'name': 'worker_0', 'pose': target}


def test_failed_motion_response_pauses_scenario_with_reason():
    result = SimpleNamespace(RESULT_OK=1, result=4, error_message='entity rejected')
    future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(result=result),
    )
    failures = []
    runner = SimpleNamespace(
        motion_futures={'worker_0': (future, 0.0, [1.0, 2.0, 0.0, 0.0])},
        action_timeout_sec=10.0,
        entities={'worker_0': {'pose': [0.0, 0.0, 0.0, 0.0]}},
        _fail_command=lambda *args: failures.append(args),
    )

    assert not ScenarioRunner._finish_motion(runner, 'worker_0')
    assert failures[0][1] == 'entity rejected'
    assert runner.entities['worker_0']['pose'] == [0.0, 0.0, 0.0, 0.0]


def test_worker_graph_crosses_both_station_transfer_routes():
    root = Path(__file__).parents[1]
    document = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())
    graph = WaypointGraph(
        document['pedestrian_waypoint_graphs']['warehouse_walkways'])

    # M01 travels between S1(-6, 3.5) and S2(-2, 3.5); M03 travels between
    # S3(2, 3.5) and S4(6, 3.5). The graph has traversable crossings within
    # both route segments, but does not put nodes in any dock footprint.
    for route_x in (-4.0, 4.0):
        crossing_nodes = [
            name for name, (x, _) in graph.nodes.items()
            if abs(x - route_x) < 0.01]
        ys = [graph.nodes[name][1] for name in crossing_nodes]
        assert min(ys) <= 2.8
        assert max(ys) >= 4.6
        assert graph.shortest_path(crossing_nodes[0], crossing_nodes[-1])


def test_roaming_is_seeded_independent_and_does_not_immediately_reverse():
    graph = WaypointGraph({
        'nodes': {'a': [0, 0], 'b': [1, 0], 'c': [1, 1], 'd': [0, 1]},
        'edges': [['a', 'b'], ['b', 'c'], ['c', 'd'], ['d', 'a']],
    })
    first = RoamingAgent('worker_a', graph, 42, (0, 0), [.4, .8], [.5, 2])
    replay = RoamingAgent('worker_a', graph, 42, (0, 0), [.4, .8], [.5, 2])
    other = RoamingAgent('worker_b', graph, 42, (0, 0), [.4, .8], [.5, 2])

    assert first.choose_trip() == replay.choose_trip()
    assert first.speed == replay.speed
    assert (first.destination, first.speed) != (other.choose_trip(), other.speed)
    origin = first.previous_origin
    first.current_node = first.destination
    first.arrive(10.0)
    assert first.choose_trip() != origin
    assert 0.4 <= first.speed <= 0.8
    first.arrive(20.0)
    assert 20.5 <= first.pause_until <= 22.0


def test_blocked_roamer_retreats_instead_of_waiting_forever():
    graph = WaypointGraph({
        'nodes': {'a': [0, 0], 'b': [1, 0], 'c': [2, 0]},
        'edges': [['a', 'b'], ['b', 'c']],
    })
    agent = RoamingAgent('worker', graph, 42, (0, 0), [.4, .8], [.5, 2])
    agent.destination = 'c'
    agent.route = ['b', 'c']

    assert not agent.wait_or_retreat(10.0)
    assert not agent.wait_or_retreat(11.9)
    assert agent.wait_or_retreat(12.0)
    assert agent.route == ['a']


def _load_warehouse_map():
    map_root = Path(__file__).parents[2] / 'navigation_bringup' / 'maps'
    metadata = yaml.safe_load((map_root / 'warehouse_map.yaml').read_text())
    tokens = [
        token for line in (map_root / metadata['image']).read_text().splitlines()
        if not line.startswith('#') for token in line.split()]
    assert tokens[0] == 'P2'
    width, height, maximum = map(int, tokens[1:4])
    pixels = list(map(int, tokens[4:]))
    assert len(pixels) == width * height
    return metadata, width, height, maximum, pixels


def test_every_roaming_edge_has_worker_radius_clearance_on_static_map():
    root = Path(__file__).parents[1]
    document = yaml.safe_load((root / 'config' / 'scenarios.yaml').read_text())
    graph_document = document['pedestrian_waypoint_graphs']['warehouse_walkways']
    graph = WaypointGraph(graph_document)
    metadata, width, height, maximum, pixels = _load_warehouse_map()
    resolution = float(metadata['resolution'])
    origin_x, origin_y = metadata['origin'][:2]

    def is_free(x, y):
        column = int((x - origin_x) / resolution)
        map_row = int((y - origin_y) / resolution)
        image_row = height - 1 - map_row
        if not (0 <= column < width and 0 <= image_row < height):
            return False
        return pixels[image_row * width + column] == maximum

    # Worker collision radius in model.sdf is 0.30 m.
    clearance_offsets = [(0.0, 0.0)] + [
        (0.30 * math.cos(index * math.pi / 8.0),
         0.30 * math.sin(index * math.pi / 8.0))
        for index in range(16)]
    for left, right in graph_document['edges']:
        x0, y0 = graph.nodes[left]
        x1, y1 = graph.nodes[right]
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, math.ceil(length / 0.05))
        for step in range(steps + 1):
            ratio = step / steps
            x = x0 + ratio * (x1 - x0)
            y = y0 + ratio * (y1 - y0)
            assert all(is_free(x + dx, y + dy)
                       for dx, dy in clearance_offsets), (
                           f'unsafe pedestrian edge {left}->{right} at {(x, y)}')


def test_all_models_are_self_contained():
    root = Path(__file__).parents[1]
    for name in ('shelf', 'pallet', 'cargo_box', 'worker', 'conveyor', 'docking_station'):
        assert (root / 'models' / name / 'model.sdf').is_file()
        assert (root / 'models' / name / 'model.config').is_file()


def test_worker_high_visibility_geometry_intersects_lidar_scan_height():
    root = Path(__file__).parents[1]
    worker = ET.parse(root / 'models' / 'worker' / 'model.sdf').getroot()
    visuals = {
        visual.attrib['name']: visual
        for visual in worker.findall('.//visual')
    }
    garment = visuals['high_visibility_apron']
    centre_z = float(garment.findtext('pose').split()[2])
    height = float(garment.findtext('.//box/size').split()[2])
    torso = visuals['torso']
    torso_centre_z = float(torso.findtext('pose').split()[2])
    torso_height = float(torso.findtext('.//box/size').split()[2])

    # Robot planar LiDAR is at z=0.35 m.  Continuous orange workwear must
    # include that plane and overlap the torso, otherwise camera segmentation
    # and physical LiDAR returns describe disjoint parts of the same worker.
    assert centre_z - height / 2 <= 0.35 <= centre_z + height / 2
    assert centre_z + height / 2 >= torso_centre_z - torso_height / 2


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
