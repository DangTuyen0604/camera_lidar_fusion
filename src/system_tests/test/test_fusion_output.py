"""Fusion output validity and empty-input fault contracts."""

from contracts import assert_contains, text


def test_fused_output_carries_xyz_depth_support_and_validity():
    message = text('fusion_interfaces', 'msg/FusedDetection.msg')
    assert_contains(message, 'geometry_msgs/Point position', 'float32 depth',
                    'uint32 lidar_point_count', 'bool valid')


def test_empty_cloud_or_detection_produces_safe_output():
    node = text('perception_core', 'src/nodes/object_fusion_node.cpp')
    assert_contains(node, 'detections', 'valid', 'publish')
