from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def share(package):
    return Path(get_package_share_directory(package))


def test_canonical_robot_and_complete_sensor_contract():
    description = share('openamrobot_description') / 'urdf'
    assert all((description / name).is_file() for name in (
        'mobile_robot.urdf.xacro', 'sensors.xacro', 'gazebo_plugins.xacro'))
    model = (description / 'robo_urdf.urdf.xacro').read_text()
    plugins = (description / 'gazebo_control.xacro').read_text()
    for frame in ('base_footprint', 'base_link', 'lidar_link', 'camera_link',
                  'camera_optical_frame'):
        assert frame in model
    for topic in ('/cmd_vel', '/odom', '/tf', '/joint_states', '/scan',
                  '/camera/image_raw'):
        assert topic in plugins


def test_nav2_has_both_obstacle_sources_and_all_servers():
    nav = share('navigation_bringup')
    params = yaml.safe_load((nav / 'config' / 'nav2_params.yaml').read_text())
    for server in ('map_server', 'amcl', 'planner_server', 'controller_server',
                   'behavior_server', 'bt_navigator', 'waypoint_follower',
                   'velocity_smoother', 'collision_monitor', 'docking_server'):
        assert server in params
    for costmap in ('local_costmap', 'global_costmap'):
        sources = params[costmap][costmap]['ros__parameters']['obstacle_layer']
        assert 'scan' in sources and 'detection_obstacles' in sources
        assert sources['detection_clearing']['clearing'] is True
    for launch_file in ('mapping.launch.py', 'localization.launch.py',
                        'navigation.launch.py', 'simulation.launch.py',
                        'full_system.launch.py', 'warehouse_full_demo.launch.py'):
        assert (nav / 'launch' / launch_file).is_file()


def test_warehouse_is_event_driven_and_missions_are_contiguous():
    warehouse = share('warehouse_simulation')
    scenarios = yaml.safe_load(
        (warehouse / 'config' / 'scenarios.yaml').read_text())['scenarios']['warehouse_demo']
    actions = {item['action'] for item in scenarios}
    assert {'wait_for_robot_region', 'wait_for_mission_state', 'spawn', 'delete',
            'follow_path', 'attach_cargo', 'detach_cargo'} <= actions
    mission = share('warehouse_mission_manager')
    missions = yaml.safe_load((mission / 'config' / 'missions.yaml').read_text())['missions']
    assert [(item['pickup'], item['destination']) for item in missions] == [
        ('S1_STORAGE', 'S2_ASSEMBLY'), ('S2_ASSEMBLY', 'S3_INSPECTION'),
        ('S3_INSPECTION', 'S4_PACKAGING'), ('S4_PACKAGING', 'S1_STORAGE')]
