"""Verify canonical frames and defensive TF lookup behavior."""

from contracts import assert_contains, text


def test_robot_tree_has_one_canonical_sensor_chain():
    model = text('openamrobot_description', 'urdf/robo_urdf.urdf.xacro')
    sensors = text('openamrobot_description', 'urdf/sensors.xacro')
    assert_contains(model, 'base_footprint', 'base_link')
    assert_contains(sensors, 'lidar_link', 'camera_link', 'camera_optical_frame')


def test_wrong_or_missing_tf_is_rejected_not_guessed():
    fusion = text('perception_core', 'src/nodes/object_fusion_node.cpp')
    bridge = text('navigation_bridge', 'src/detection_obstacle_bridge_node.cpp')
    assert_contains(fusion, 'lookupTransform', 'TransformException')
    assert_contains(bridge, 'lookupTransform', 'TransformException')
