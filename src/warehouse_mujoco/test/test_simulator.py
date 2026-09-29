"""Headless checks of the MuJoCo warehouse (no ROS, no OpenGL)."""

import math
from pathlib import Path

import numpy as np
import pytest

from warehouse_mujoco import sdf_loader
from warehouse_mujoco.simulator import WarehouseSim

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
WAREHOUSE = SRC / 'warehouse_simulation'
MESHES = SRC / 'openamrobot_description' / 'meshes' / 'visual'


@pytest.fixture(scope='module')
def sim():
    models = WAREHOUSE / 'models'
    return WarehouseSim(
        PACKAGE / 'mjcf' / 'scene.xml', WAREHOUSE / 'worlds' / 'warehouse.sdf',
        [models], MESHES,
        {name: models / name / 'model.sdf' for name in ('worker', 'cargo_box', 'pallet')},
        pool_size=2)


def test_sdf_pose_matches_fixed_axis_rpy():
    _, quat = sdf_loader.parse_pose('0 0 0 0 0 1.5707963')
    assert np.allclose(quat, [math.sqrt(0.5), 0, 0, math.sqrt(0.5)])


def test_world_converted_from_sdf(sim):
    names = {sim.model.geom(i).name for i in range(sim.model.ngeom)}
    assert 'shelf_west_1/link/collision' in names
    assert 'dock_home/link/collision' in names
    assert sim.ground_z == pytest.approx(0.05)


def test_robot_rests_on_floor_at_home(sim):
    sim.reset(0.0, -4.0, 0.0)
    sim.step(500)
    x, y, yaw = sim.robot_pose()
    assert (x, y, yaw) == pytest.approx((0.0, -4.0, 0.0), abs=0.01)
    assert sim.data.qpos[2] == pytest.approx(sim.ground_z + 0.0635, abs=0.003)


def test_wheel_odometry_tracks_ground_truth(sim):
    sim.reset(0.0, 0.0, 0.0)  # open aisle: the arc touches no obstacle
    sim.step(300)
    start = np.array(sim.robot_pose())
    sim.set_cmd_vel(0.5, -0.3)
    sim.step(3000)
    sim.set_cmd_vel(0.0, 0.0)
    sim.step(1000)
    truth = np.array(sim.robot_pose()) - start
    assert np.hypot(*truth[:2]) > 1.5
    assert np.hypot(*(sim.odom[:2] - truth[:2])) < 0.03
    assert abs(sim.odom[2] - truth[2]) < 0.03


def test_robot_turns_in_place_at_home_dock(sim):
    """HOME leaves 6 cm to the dock; turning there must not snag or slip."""
    sim.reset(0.0, -4.0, 0.0)
    sim.step(300)
    sim.set_cmd_vel(0.0, -0.07)  # Nav2 rotation, slowed by the SlowZone
    sim.step(5000)
    x, y, yaw = sim.robot_pose()
    assert yaw < -0.6
    assert abs(sim.odom[2] - yaw) < 0.04  # ~3 % in-place scrub
    assert math.hypot(x, y + 4.0) < 0.01


def test_velocity_and_acceleration_limits(sim):
    sim.reset(0.0, -4.0, 0.0)
    sim.set_cmd_vel(5.0, -9.0)
    assert tuple(sim.cmd) == (1.0, -2.0)
    sim.step(500)  # 1 s at 0.5 m/s^2
    assert sim.applied[0] == pytest.approx(0.5, abs=1e-6)


def test_lidar_sees_dock_and_spawned_worker(sim):
    sim.reset(0.0, -4.7, math.pi / 2)  # facing the HOME dock at y=-3.65
    sim.step(200)
    ranges = sim.scan()
    front = ranges[np.argmin(np.abs(sim.lidar_angles))]
    assert front == pytest.approx(-3.65 - -4.7, abs=0.03)  # lidar_link at x=0
    behind = ranges[0]
    assert np.isfinite(behind)
    sim.spawn_entity('worker_a', 'worker', [0.0, -5.9, 0.0], [1, 0, 0, 0])
    sim.step(1)
    assert sim.scan()[0] < behind - 0.5
    sim.delete_entity('worker_a')
    sim.step(1)
    assert sim.scan()[0] == pytest.approx(behind, abs=0.02)


def test_entity_pool_lifecycle(sim):
    sim.reset(0.0, -4.0, 0.0)
    sim.spawn_entity('a', 'pallet', [1, 1, 0.1], [1, 0, 0, 0])
    sim.spawn_entity('b', 'pallet', [2, 1, 0.1], [1, 0, 0, 0])
    with pytest.raises(RuntimeError):
        sim.spawn_entity('c', 'pallet', [3, 1, 0.1], [1, 0, 0, 0])
    with pytest.raises(ValueError):
        sim.spawn_entity('a', 'worker', [0, 0, 0], [1, 0, 0, 0])
    sim.set_entity_pose('a', [4, 1, 0.1], [1, 0, 0, 0])
    assert sim.entity_pose('a')[0] == pytest.approx([4, 1, 0.1])
    sim.delete_entity('a')
    sim.spawn_entity('c', 'pallet', [3, 1, 0.1], [1, 0, 0, 0])
