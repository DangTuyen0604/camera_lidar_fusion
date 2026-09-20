"""Fused-detection to costmap-obstacle pipeline contract."""

from contracts import assert_contains, text, yaml_asset


def test_bridge_filters_and_costmaps_consume_obstacles():
    bridge = text('navigation_bridge', 'src/detection_obstacle_bridge_node.cpp')
    assert_contains(bridge, 'minimum_confidence', 'minimum_range',
                    'maximum_range', 'obstacle_timeout')
    params = yaml_asset('navigation_bringup', 'config/nav2_params.yaml')
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['obstacle_layer']
        assert layer['detection_obstacles']['topic'] == (
            '/navigation/detection_obstacles')
        assert layer['detection_obstacles']['marking'] is True
        assert layer['detection_clearing']['topic'] == (
            '/navigation/detection_obstacles/clearing')
        assert layer['detection_clearing']['clearing'] is True
