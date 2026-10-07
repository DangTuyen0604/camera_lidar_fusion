"""Semantic configuration checks for the Stage 3 navigation safety chain."""

import ast
import importlib.util
from pathlib import Path

from contracts import package_root, text, yaml_asset


def test_stage3_uses_clean_geometry_map():
    """The MuJoCo demo must not load scan-shadow artifacts from the SLAM map."""
    launch = text('navigation_bringup', 'launch/stage3.launch.py')

    assert "'maps' / 'warehouse_map.yaml'" in launch
    assert "'maps' / 'warehouse_slam_map.yaml'" not in launch


def test_geometry_map_initial_pose_matches_warehouse_spawn():
    """AMCL and the simulator must agree on the robot's initial map coordinate."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    initial = config['amcl']['ros__parameters']['initial_pose']
    warehouse = text(
        'warehouse_simulation', 'launch/warehouse.launch.py')

    assert initial['x'] == 0.0
    assert initial['y'] == -4.0
    assert "'spawn_x': '0.0'" in warehouse
    assert "'spawn_y': '-4.0'" in warehouse


def _warehouse_map(name='warehouse_map'):
    maps = package_root('navigation_bringup') / 'maps'
    metadata = yaml_asset('navigation_bringup', f'maps/{name}.yaml')
    tokens = [
        token for line in (maps / metadata['image']).read_text().splitlines()
        if not line.startswith('#') for token in line.split()
    ]
    assert tokens[0] == 'P2' and tokens[3] == '9'
    width, height = int(tokens[1]), int(tokens[2])
    pixels = [int(value) for value in tokens[4:]]
    assert len(pixels) == width * height
    return metadata, width, height, pixels


def test_warehouse_map_resolves_docking_precision():
    """AMCL cannot resolve sub-cell error; docking is validated to 8 cm."""
    metadata, _, _, _ = _warehouse_map()
    assert metadata['resolution'] <= 0.05


def test_warehouse_map_is_generated_from_current_sdf():
    """Re-rasterize warehouse.sdf; edit the world, then rerun the generator."""
    workspace = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'generate_warehouse_map', workspace / 'tools' / 'generate_warehouse_map.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    metadata, width, height, pixels = _warehouse_map()
    expected = generator.rasterize(metadata['resolution'])
    assert expected.shape == (height, width)
    assert expected.ravel().tolist() == pixels, (
        'warehouse_map.pgm is stale: run tools/generate_warehouse_map.py')


def _generator():
    workspace = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'generate_warehouse_map', workspace / 'tools' / 'generate_warehouse_map.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    return generator


def test_amcl_localizes_on_the_scan_plane_map():
    """
    AMCL must match scans against what the LiDAR can actually see.

    The conveyor belt (0.64-1.00 m) is above the scan plane: it stays in /map
    for the planner but must not appear in AMCL's map, which also compensates
    nav2_amcl's half-cell origin convention. Without both, dock poses were
    localized up to 4 cm off (4 deg physical yaw at S1).
    """
    generator = _generator()
    metadata, width, height, pixels = _warehouse_map('warehouse_localization_map')
    resolution = metadata['resolution']
    assert resolution <= 0.025
    nav = yaml_asset('navigation_bringup', 'maps/warehouse_map.yaml')
    assert metadata['origin'][0] == nav['origin'][0] + resolution / 2
    assert metadata['origin'][1] == nav['origin'][1] + resolution / 2
    expected = generator.rasterize(resolution, generator.SCAN_HEIGHT)
    assert expected.shape == (height, width)
    assert expected.ravel().tolist() == pixels, (
        'warehouse_localization_map.pgm is stale: run tools/generate_warehouse_map.py')

    def occupied(map_metadata, map_width, map_height, map_pixels, x, y):
        res = map_metadata['resolution']
        column = int((x - generator.ORIGIN[0]) / res)
        row = map_height - 1 - int((y - generator.ORIGIN[1]) / res)
        return map_pixels[row * map_width + column] == 0

    belt = (-2.0, 4.5)
    assert occupied(*_warehouse_map(), *belt)
    assert not occupied(metadata, width, height, pixels, *belt)
    assert occupied(metadata, width, height, pixels, -5.55, 3.5)  # dock S1 body

    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    assert config['amcl']['ros__parameters']['map_topic'] == 'localization_map'
    launch = text('navigation_bringup', 'launch/localization.launch.py')
    assert "'warehouse_localization_map.yaml'" in launch
    assert "'topic_name': 'localization_map'" in launch
    assert "'localization_map_server'" in launch.split('node_names')[1]


def test_warehouse_map_matches_all_physical_docks():
    """The static map must place every dock where simulator collision geometry is."""
    metadata, width, height, pixels = _warehouse_map()
    resolution = metadata['resolution']
    origin_x, origin_y = metadata['origin'][:2]

    def occupied(world_x, world_y):
        column = int((world_x - origin_x) / resolution)
        row = height - 1 - int((world_y - origin_y) / resolution)
        return pixels[row * width + column] == 0

    # Collision centres include the model-local -0.15 m pose offset.
    for centre in ((-5.5, 3.5), (-1.5, 3.5),
                   (1.5, 3.5), (5.5, 3.5), (0.0, -3.5)):
        assert occupied(*centre), f'dock missing from map at {centre}'

    # These were phantom dock locations in the previous, mismatched map.
    assert not occupied(-7.0, 4.0)
    assert not occupied(7.0, 4.0)
    assert not occupied(0.0, -5.0)


def test_collision_monitor_owns_final_velocity_command():
    """Collision Monitor must sit between the smoother and simulator command."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert monitor['scan']['topic'] == '/scan_filtered'
    assert monitor['detection_obstacles']['topic'] == (
        '/navigation/detection_obstacles')
    assert monitor['detection_obstacles']['type'] == 'pointcloud'
    assert monitor['base_frame_id'] == 'base_link'
    assert monitor['odom_frame_id'] == 'odom'


def test_safety_zones_are_ordered_and_actionable():
    """Safety must slow early and project the footprint before collision."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']
    stop = monitor['StopZone']
    slow = ast.literal_eval(monitor['SlowZone']['points'])
    assert stop['type'] == 'polygon'
    assert stop['action_type'] == 'approach'
    assert stop['footprint_topic'] == '/local_costmap/published_footprint'
    assert stop['time_before_collision'] > 0.0
    assert 0.0 < stop['simulation_time_step'] <= 0.1
    assert monitor['SlowZone']['action_type'] == 'slowdown'
    assert 0.0 < monitor['SlowZone']['slowdown_ratio'] < 1.0
    assert max(x for x, _ in slow) >= 1.0
    assert max(abs(y) for _, y in slow) >= 0.45
    assert stop['min_points'] > 0
    assert monitor['SlowZone']['min_points'] > 0


def test_controller_can_turn_onto_replanned_path():
    """RPP must be able to rotate after a dynamic obstacle changes the path."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    controller = config['controller_server']['ros__parameters']['FollowPath']

    assert controller['use_rotate_to_heading'] is True
    assert controller['rotate_to_heading_angular_vel'] > 0.0
    assert 0.0 < controller['rotate_to_heading_min_angle'] < 1.57
    progress = config['controller_server']['ros__parameters'][
        'progress_checker']
    assert progress['movement_time_allowance'] >= 30.0


def test_detection_observations_do_not_leave_trails():
    """
    Moving workers must not leave stale marks in either costmap.

    Nav2 keeps the newest observation until it is replaced when persistence
    is 0, so the 2 Hz planner map still sees every obstacle. A persistence
    window instead re-marks old batches after the bridge's clearing rays ran.
    """
    config = yaml_asset('navigation_bringup', 'config/nav2_stage3_params.yaml')
    for name in ('local_costmap', 'global_costmap'):
        parameters = config[name][name]['ros__parameters']
        layer = parameters['obstacle_layer']
        source = layer['detection_obstacles']
        assert source['observation_persistence'] == 0.0
        clearing = layer['detection_clearing']
        assert clearing.get('observation_persistence', 0.0) == 0.0
        # Nav2's per-source max_obstacle_height default (0.0) drops
        # every clearing point of a box or pallet above the floor.
        assert clearing['min_obstacle_height'] <= 0.0
        assert clearing['max_obstacle_height'] >= source['max_obstacle_height']
        assert source['min_obstacle_height'] <= 0.05
        assert source['max_obstacle_height'] >= 2.0


def test_mission_detections_expire_instead_of_waiting_for_clearing():
    """
    The mission stack marks detections in a decaying voxel layer.

    Every cell expires shortly after its last detection, so a worker cannot
    leave a footprint behind when one clearing batch is missed.
    """
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    for name in ('local_costmap', 'global_costmap'):
        parameters = config[name][name]['ros__parameters']
        plugins = parameters['plugins']
        assert plugins.index('detection_layer') < plugins.index('inflation_layer')
        assert parameters['obstacle_layer']['observation_sources'] == 'scan'
        layer = parameters['detection_layer']
        assert layer['plugin'] == (
            'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
        assert layer['decay_model'] == 0
        # Long enough to survive one planner cycle, short enough that a
        # walking worker leaves at most a short, self-clearing tail.
        assert 1.0 / parameters['update_frequency'] <= layer['voxel_decay'] <= 1.0
        source = layer[layer['observation_sources']]
        assert source['topic'] == '/navigation/detection_obstacles'
        assert source['marking'] is True and source['clearing'] is False
        assert source['min_obstacle_height'] <= 0.05
        assert source['max_obstacle_height'] >= 2.0
        assert source['obstacle_range'] >= 30.0
    # A stationary robot's fixed beam angles cannot raytrace-clear every cell
    # a passing worker was marked in, so local LiDAR marks must expire too.
    local = config['local_costmap']['local_costmap']['ros__parameters']
    scan_layer = local['obstacle_layer']
    assert scan_layer['plugin'] == (
        'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
    assert 0.0 < scan_layer['voxel_decay'] <= 1.0
    assert scan_layer['scan']['topic'] == '/scan_filtered'


def test_mission_planner_map_marks_lidar_obstacles():
    """
    The planner must see LiDAR obstacles the controller refuses to drive into.

    Nav2's per-source max_obstacle_height default (0.0) silently drops every
    return of the 0.35 m LiDAR; the planner then kept routing M02 through the
    fallen box that the local costmap held as lethal.
    """
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    parameters = config['global_costmap']['global_costmap']['ros__parameters']
    source = parameters['obstacle_layer']['scan']
    assert source['marking'] is True
    assert source['min_obstacle_height'] <= 0.0
    assert source['max_obstacle_height'] >= 2.0


def test_mission_staging_is_tight_enough_for_docking():
    """
    Staging error becomes dock error over the short 0.70 m approach.

    With 0.10 m / 0.25 rad, S3/S4 docked at 4-5 deg against the 5 deg contract.
    """
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    checker = config['controller_server']['ros__parameters']['goal_checker']
    assert checker['xy_goal_tolerance'] <= 0.05
    assert checker['yaw_goal_tolerance'] <= 0.10
    manager = text('warehouse_mission_manager',
                   'warehouse_mission_manager/mission_manager.py')
    assert '_start_dock_when_settled' in manager


def test_navigation_consumers_share_the_filtered_lidar():
    """Localization, both costmaps, and safety must use one filtered scan."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    topics = {
        config['amcl']['ros__parameters']['scan_topic'],
        config['local_costmap']['local_costmap']['ros__parameters'][
            'obstacle_layer']['scan']['topic'],
        config['global_costmap']['global_costmap']['ros__parameters'][
            'obstacle_layer']['scan']['topic'],
        config['collision_monitor']['ros__parameters']['scan']['topic'],
    }
    assert topics == {'/scan_filtered'}


def test_warehouse_navigation_has_one_safety_enforced_velocity_path():
    """Controller and docking commands must both pass through safety."""
    launch = text('navigation_bringup', 'launch/navigation.launch.py')
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']

    assert "remappings=[('cmd_vel', 'cmd_vel_nav')]" in launch
    assert launch.count("remappings=[('cmd_vel', 'cmd_vel_nav')]") == 4
    # smoother -> dynamic_yield (stop-and-wait) -> Collision Monitor -> robot
    assert ("remappings=[('cmd_vel_in', 'cmd_vel_smoothed'),\n"
            "                         ('cmd_vel_out', 'cmd_vel_yield')]") in launch
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_yield'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert set(monitor['observation_sources']) == {
        'scan', 'detection_obstacles'}
    assert monitor['scan']['topic'] == '/scan_filtered'
    assert monitor['detection_obstacles']['topic'] == (
        '/navigation/detection_obstacles')


def test_warehouse_localization_and_costmaps_share_filtered_lidar():
    """AMCL and both costmaps must consume the same body-filtered scan."""
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    topics = {
        config['amcl']['ros__parameters']['scan_topic'],
        config['local_costmap']['local_costmap']['ros__parameters'][
            'obstacle_layer']['scan']['topic'],
        config['global_costmap']['global_costmap']['ros__parameters'][
            'obstacle_layer']['scan']['topic'],
        config['collision_monitor']['ros__parameters']['scan']['topic'],
    }
    assert topics == {'/scan_filtered'}


def test_rviz_costmaps_use_latched_nav2_qos():
    """Late-starting RViz must receive Nav2's transient-local full maps."""
    rviz = yaml_asset('navigation_bringup', 'rviz/navigation.rviz')
    displays = rviz['Visualization Manager']['Displays']
    by_name = {display.get('Name'): display for display in displays}

    for name in ('Local Costmap', 'Global Costmap'):
        display = by_name[name]
        assert display['Topic']['Durability Policy'] == 'Transient Local'
        assert display['Update Topic']['Durability Policy'] == (
            'Transient Local')
    # LaserScan is a live stream and must remain compatible with volatile QoS.
    assert by_name['Live LiDAR Scan']['Topic']['Durability Policy'] == (
        'Volatile')


def test_all_stage3_lifecycle_nodes_are_managed():
    """Every required navigation and safety lifecycle node must autostart."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    manager = config['lifecycle_manager_navigation']['ros__parameters']
    assert manager['autostart'] is True
    assert set(manager['node_names']) == {
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    }


def test_costmaps_see_obstacles_below_the_scan_plane():
    """
    Both costmaps must mark ground-removed 3D LiDAR points.

    The 0.18 m blocking pallet is under the 0.46 m planar scan; only the
    camera layer saw it, and the robot pressed into it until its wheels
    slipped and AMCL drifted 0.5 m. Marks under the 16-ring frustum must
    outlive the moment the pallet drops into the lowest beam's blind zone.
    """
    config = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    for name in ('local_costmap', 'global_costmap'):
        parameters = config[name][name]['ros__parameters']
        plugins = parameters['plugins']
        assert plugins.index('lidar_layer') < plugins.index('inflation_layer')
        layer = parameters['lidar_layer']
        assert layer['plugin'] == 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer'
        mark, clear = (layer[source] for source in layer['observation_sources'].split())
        assert mark['topic'] == clear['topic'] == '/lidar/obstacles'
        assert mark['marking'] and not mark['clearing']
        assert mark['min_obstacle_height'] <= 0.05 < 0.18
        assert clear['clearing'] and clear['model_type'] == 1
        assert layer['voxel_decay'] >= 5.0
        assert layer['update_footprint_enabled']

    launch = text('navigation_bringup', 'launch/navigation.launch.py')
    assert "executable='pointcloud_preprocess_node'" in launch
    assert "'sim_vlp16.yaml'" in launch
    profile = yaml_asset('perception_core', 'config/sim_vlp16.yaml')
    preprocess = profile['pointcloud_preprocess']['ros__parameters']
    assert preprocess['input_topic'] == '/lidar/points_3d'
    # STVL takes the frustum origin from the message frame.
    assert preprocess['output_frame'] == 'sensor'
    assert preprocess['ground.enabled']
    simulation = text('warehouse_simulation', 'launch/warehouse.launch.py')
    assert "'lidar_3d_channels', default_value='16'" in simulation
