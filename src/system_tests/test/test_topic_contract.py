"""Verify the full-system topic and message-type contract."""

from contracts import assert_contains, text


def test_perception_navigation_and_mission_topics_are_typed():
    interfaces = ''.join(text('fusion_interfaces', f'msg/{name}.msg') for name in (
        'Detection2DArray', 'FusedDetectionArray', 'PipelineMetrics',
        'CalibrationStatus', 'SyncStatus'))
    assert_contains(interfaces, 'Detection2D[] detections',
                    'FusedDetection[] detections', 'float32 end_to_end_p95_ms',
                    'float32 projection_error_px', 'uint64 dropped_messages')
    bridge = text('navigation_bridge', 'src/detection_obstacle_bridge_node.cpp')
    assert_contains(bridge, '/fusion/detections_3d',
                    '/navigation/detection_obstacles')
    mission = text('warehouse_mission_manager',
                   'warehouse_mission_manager/mission_manager.py')
    assert_contains(mission, '/mission/state', '/mission/cargo', '/odom')
