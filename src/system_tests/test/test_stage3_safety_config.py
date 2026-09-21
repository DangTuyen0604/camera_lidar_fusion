"""Semantic configuration checks for the Stage 3 navigation safety chain."""

import ast

from contracts import yaml_asset


def test_collision_monitor_owns_final_velocity_command():
    """Collision Monitor must sit between the smoother and Gazebo command."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel'
    assert monitor['scan']['topic'] == '/scan_filtered'
    assert monitor['base_frame_id'] == 'base_link'
    assert monitor['odom_frame_id'] == 'odom'


def test_safety_zones_are_ordered_and_actionable():
    """The stop polygon must be contained inside a larger slowdown polygon."""
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']
    stop = ast.literal_eval(monitor['StopZone']['points'])
    slow = ast.literal_eval(monitor['SlowZone']['points'])
    assert monitor['StopZone']['action_type'] == 'stop'
    assert monitor['SlowZone']['action_type'] == 'slowdown'
    assert 0.0 < monitor['SlowZone']['slowdown_ratio'] < 1.0
    assert max(x for x, _ in slow) > max(x for x, _ in stop)
    assert max(abs(y) for _, y in slow) > max(abs(y) for _, y in stop)
    assert monitor['StopZone']['min_points'] > 0
    assert monitor['SlowZone']['min_points'] > 0


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
