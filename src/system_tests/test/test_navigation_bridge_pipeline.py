"""Fused-detection to costmap-obstacle pipeline contract."""

from contracts import assert_contains, text, yaml_asset


def test_bridge_filters_and_costmaps_consume_obstacles():
    bridge = text('navigation_bridge', 'src/detection_obstacle_bridge_node.cpp')
    assert_contains(bridge, 'minimum_confidence', 'minimum_range',
                    'maximum_range', 'obstacle_timeout')
    for config in ('nav2_params.yaml', 'nav2_stage3_params.yaml'):
        params = yaml_asset('navigation_bringup', f'config/{config}')
        for name in ('local_costmap', 'global_costmap'):
            costmap = params[name][name]['ros__parameters']
            if 'detection_layer' in costmap['plugins']:
                # Mission stack: detections expire in a decaying voxel layer.
                layer = costmap['detection_layer']
                source = layer[layer['observation_sources']]
                assert source['topic'] == '/navigation/detection_obstacles'
                assert source['marking'] is True
                continue
            layer = costmap['obstacle_layer']
            assert layer['detection_obstacles']['topic'] == (
                '/navigation/detection_obstacles')
            assert layer['detection_obstacles']['marking'] is True
            assert layer['detection_clearing']['topic'] == (
                '/navigation/detection_obstacles/clearing')
            assert layer['detection_clearing']['clearing'] is True
