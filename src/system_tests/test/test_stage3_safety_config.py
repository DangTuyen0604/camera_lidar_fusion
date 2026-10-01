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


def _warehouse_map():
    maps = package_root('navigation_bringup') / 'maps'
    metadata = yaml_asset('navigation_bringup', 'maps/warehouse_map.yaml')
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
    """Moving workers must not leave stale marks in either costmap.

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
    """The mission stack marks detections in a decaying voxel layer.

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
