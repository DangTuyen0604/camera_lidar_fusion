from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from warehouse_mission_manager.cargo_manager import CargoManager
from warehouse_mission_manager.docking_controller import DockingController
from warehouse_mission_manager.mission_manager import (
    MissionManager, navigation_succeeded)
from warehouse_mission_manager.mission_state import MissionState
import pytest
import yaml


def test_missions_and_state_machine_contract():
    root = Path(__file__).parents[1]
    missions = yaml.safe_load((root / 'config' / 'missions.yaml').read_text())['missions']
    assert [mission['id'] for mission in missions] == ['M01', 'M02', 'M03', 'M04']
    assert [state.value for state in MissionState] == [
        'IDLE', 'ACCEPTED', 'GO_TO_PICKUP', 'WAIT_FOR_LOADING', 'CARGO_LOADED',
        'GO_TO_PRE_DOCK', 'DOCKING', 'DOCKED', 'UNLOADING', 'UNDOCKING',
        'COMPLETED', 'NEXT_MISSION']


def test_cargo_invariant():
    cargo = CargoManager()
    cargo.load('M01_cargo')
    assert cargo.loaded and cargo.unload() == 'M01_cargo' and not cargo.loaded


def test_docking_acceptance_limits():
    pose = SimpleNamespace(position=SimpleNamespace(x=1.04, y=2.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    twist = SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0),
                            angular=SimpleNamespace(z=0.0))
    target = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    assert DockingController.validate(pose, twist, target, False)
    assert not DockingController.validate(pose, twist, target, True)
    pose.position.x = 1.09
    assert not DockingController.validate(pose, twist, target, False)


def test_undock_returns_to_staging_before_next_mission():
    callbacks = []
    goals = []
    result_future = SimpleNamespace(add_done_callback=lambda callback: callback(
        SimpleNamespace(result=lambda: SimpleNamespace(result=SimpleNamespace(
            success=True, error_code=0, error_msg='')))))
    handle = SimpleNamespace(
        accepted=True, get_result_async=lambda: result_future)
    send_future = SimpleNamespace(add_done_callback=lambda callback: callback(
        SimpleNamespace(result=lambda: handle)))
    controller = DockingController.__new__(DockingController)
    controller.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        info=lambda *args: None, warn=lambda *args: None,
        error=lambda *args: None))
    controller.undock_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal: goals.append(goal) or send_future)

    assert controller.undock(callbacks.append)
    assert goals[0].dock_type == 'warehouse_dock'
    assert goals[0].max_undocking_time == 60.0
    assert callbacks == [True]


def test_docking_collision_check_frame_matches_rolling_costmap():
    root = Path(__file__).parents[2]
    parameters = yaml.safe_load((
        root / 'navigation_bringup' / 'config' / 'nav2_params.yaml').read_text())
    costmap_frame = parameters[
        'local_costmap']['local_costmap']['ros__parameters']['global_frame']
    docking_frame = parameters[
        'docking_server']['ros__parameters']['fixed_frame']
    recovery_frame = parameters[
        'behavior_server']['ros__parameters']['local_frame']
    assert costmap_frame == docking_frame == recovery_frame == 'map'


def test_station_and_docking_server_coordinates_match():
    root = Path(__file__).parents[2]
    stations = yaml.safe_load((
        Path(__file__).parents[1] / 'config' / 'stations.yaml').read_text())[
            'stations']
    docks = yaml.safe_load((
        root / 'navigation_bringup' / 'config' / 'nav2_params.yaml').read_text())[
            'docking_server']['ros__parameters']
    for name, station in stations.items():
        assert docks[name]['frame'] == 'map'
        assert docks[name]['pose'] == [
            station['x'], station['y'], station['yaw']]


def test_s2_staging_pose_and_footprint_are_inside_map():
    root = Path(__file__).parents[2]
    map_config = yaml.safe_load((
        root / 'navigation_bringup' / 'maps' /
        'warehouse_map.yaml').read_text())
    nav = yaml.safe_load((
        root / 'navigation_bringup' / 'config' / 'nav2_params.yaml').read_text())
    dock = nav['docking_server']['ros__parameters']
    x, y, yaw = dock['S2_ASSEMBLY']['pose']
    offset = dock['warehouse_dock']['staging_x_offset']
    staging = (x + offset * math.cos(yaw), y + offset * math.sin(yaw))

    header = [
        token for line in (
            root / 'navigation_bringup' / 'maps' / map_config['image']
        ).read_text().splitlines()[:4]
        if not line.startswith('#') for token in line.split()]
    width, height = int(header[1]), int(header[2])
    origin_x, origin_y, _ = map_config['origin']
    max_x = origin_x + width * map_config['resolution']
    max_y = origin_y + height * map_config['resolution']
    radius = nav['local_costmap']['local_costmap']['ros__parameters'][
        'robot_radius']
    for pose_x, pose_y in ((x, y), staging):
        assert origin_x + radius < pose_x < max_x - radius
        assert origin_y + radius < pose_y < max_y - radius
    assert staging == pytest.approx((-2.97, 3.5))


def test_docking_contact_zone_matches_robot_geometry():
    root = Path(__file__).parents[2]
    nav = yaml.safe_load((
        root / 'navigation_bringup' / 'config' / 'nav2_params.yaml').read_text())
    docking = nav['docking_server']['ros__parameters']
    radius = nav['local_costmap']['local_costmap']['ros__parameters'][
        'robot_radius']
    contact_zone = docking['controller']['dock_collision_threshold']
    staging_distance = abs(docking['warehouse_dock']['staging_x_offset'])

    # Collision prediction remains active throughout the free approach, then
    # yields only where physical contact with the dock is expected.
    assert radius <= contact_zone < staging_distance
    assert (docking['warehouse_dock']['docking_threshold'] <=
            DockingController.POSITION_TOLERANCE - 0.02)


def test_docking_validation_transforms_odometry_pose_to_map():
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.orientation.w = 1.0
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'
    transform.transform.translation.x = -3.0
    transform.transform.translation.y = 3.5
    transform.transform.rotation.w = 1.0
    fake = SimpleNamespace(
        odom=odom,
        tf_buffer=SimpleNamespace(
            lookup_transform=lambda *args, **kwargs: transform),
        get_logger=lambda: SimpleNamespace(error=lambda *args: None),
    )

    map_pose = MissionManager._robot_pose_in_map(fake)
    assert map_pose.position.x == -2.0
    assert map_pose.position.y == 3.5
    assert DockingController.validate(
        map_pose, odom.twist.twist,
        {'x': -2.0, 'y': 3.5, 'yaw': 0.0}, False)


def test_navigation_result_rejects_canceled_and_aborted_goals():
    result = SimpleNamespace(error_code=0)
    assert navigation_succeeded(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result))
    assert not navigation_succeeded(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=result))
    assert not navigation_succeeded(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED, result=result))


def test_manifest_declares_navigation_readiness_dependencies():
    root = Path(__file__).parents[1]
    package = ET.parse(root / 'package.xml').getroot()
    dependencies = {item.text for item in package.findall('exec_depend')}
    assert {'ament_index_python', 'lifecycle_msgs', 'tf2_geometry_msgs',
            'tf2_ros'} <= dependencies
