"""Static contracts for the live Gate 11 perception-navigation path."""

from contracts import assert_contains, text, yaml_asset


def test_gate11_launch_uses_sensor_detection_fusion_and_navigation():
    launch = text(
        'fusion_bringup', 'launch/final_demo.launch.py')
    assert_contains(
        launch, 'stage3.launch.py', 'sensor_sync_node',
        'simulation_color_detector', 'object_fusion_node', 'metrics_node')
    assert '/camera/image_raw' in launch
    assert '/lidar/points' in launch
    assert 'scenario_runner' not in launch


def test_collision_monitor_consumes_bridge_output():
    config = yaml_asset(
        'navigation_bringup', 'config/nav2_stage3_params.yaml')
    monitor = config['collision_monitor']['ros__parameters']
    assert 'detection_obstacles' in monitor['observation_sources']
    source = monitor['detection_obstacles']
    assert source['type'] == 'pointcloud'
    assert source['topic'] == '/navigation/detection_obstacles'
    assert source['enabled'] is True


def test_warehouse_bringup_uses_live_perception_not_scenario_truth():
    """The integrated mission must have one real producer for fused XYZ."""
    bringup = text('fusion_bringup', 'launch/bringup_sim.launch.py')
    warehouse_launch = text(
        'warehouse_simulation', 'launch/warehouse.launch.py')
    scenario = text(
        'warehouse_simulation', 'warehouse_simulation/scenario_runner.py')

    assert_contains(
        bringup,
        "'publish_fused_detections': 'false'",
        "executable='sensor_sync_node'",
        "executable='simulation_color_detector'",
        "executable='object_fusion_node'",
        "executable='metrics_node'",
    )
    assert 'publish_fused_detections' in warehouse_launch
    assert "'/benchmark/ground_truth/detections_3d'" in scenario
    assert 'if self.detection_publisher is not None' in scenario
