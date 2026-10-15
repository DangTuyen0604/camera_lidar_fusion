"""Headless checks of the MuJoCo warehouse (no ROS, no OpenGL)."""

import math
from pathlib import Path

import numpy as np
import pytest

from warehouse_mujoco import sdf_loader
from warehouse_mujoco.simulator import EncoderConfig, ImuConfig, Lidar3dConfig, WarehouseSim

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


def test_3d_lidar_sees_floor_and_low_pallet_the_planar_scan_misses():
    models = WAREHOUSE / 'models'
    sim = WarehouseSim(
        PACKAGE / 'mjcf' / 'scene.xml', WAREHOUSE / 'worlds' / 'warehouse.sdf',
        [models], MESHES, {'pallet': models / 'pallet' / 'model.sdf'}, pool_size=1,
        lidar_3d=Lidar3dConfig())
    sim.reset(0.0, 0.0, 0.0)  # centre aisle, facing +x
    sim.step(200)
    sim.spawn_entity('pallet_a', 'pallet', [2.0, 0.0, 0.10], [1, 0, 0, 0])  # 0.18 m tall
    sim.step(1)

    points, rings = sim.scan_3d()
    assert len(points) == len(rings) > 0
    origin = sim.data.site_xpos[sim.lidar_site]
    rotation = sim.data.site_xmat[sim.lidar_site].reshape(3, 3)
    world = points.astype(float) @ rotation.T + origin
    # The lowest ring reaches the floor; nothing is below it.
    floor = world[rings == 0]
    assert np.median(np.abs(floor[:, 2] - sim.ground_z)) < 0.03
    assert world[:, 2].min() > sim.ground_z - 0.05
    # The pallet (x 1.4..2.6, |y| < 0.4, top 0.18 m) is in the 3D cloud ...
    on_pallet = ((world[:, 0] > 1.35) & (world[:, 0] < 2.65) & (np.abs(world[:, 1]) < 0.45)
                 & (world[:, 2] > sim.ground_z + 0.03) & (world[:, 2] < sim.ground_z + 0.25))
    assert on_pallet.sum() >= 10
    # ... but the 0.47 m scan plane passes over it.
    front = sim.scan()[np.argmin(np.abs(sim.lidar_angles))]
    assert front > 3.0


@pytest.fixture(scope='module')
def robot_sensors_sim():
    """Build a simulator with the OpenAMRobot's sensors (MPU-6500, AS5040 encoders)."""
    return WarehouseSim(
        PACKAGE / 'mjcf' / 'scene.xml', WAREHOUSE / 'worlds' / 'warehouse.sdf',
        [WAREHOUSE / 'models'], MESHES, seed=3, imu=ImuConfig.mpu6500(),
        encoder=EncoderConfig.as5040())


def imu_samples(sim, seconds, period=0.02):
    samples = []
    for _ in range(round(seconds / period)):
        sim.step(round(period / sim.model.opt.timestep))
        samples.append(sim.imu_sample())
    return samples


def test_mpu6500_firmware_removes_the_boot_offset_and_hides_slow_rotation(robot_sensors_sim):
    sim = robot_sensors_sim
    sim.reset(0.0, -5.0, 0.0)
    # The boot calibration measures the turn-on offset to within its averaged noise...
    assert np.allclose(sim.gyro_offset, sim.gyro_bias, atol=math.radians(0.05))
    sim.step(250)
    sim.imu_sample()
    resting = imu_samples(sim, 1.0)
    # ...so at rest the gyro reads exactly 0.
    assert all(np.all(gyro == 0.0) for _, gyro, _ in resting)
    lsb = 9.80665 / 16384.0
    for _, _, accel in resting:
        assert accel[2] == pytest.approx(9.81, abs=0.3)
        assert accel[2] / lsb == pytest.approx(round(accel[2] / lsb), abs=1e-6)

    sim.set_cmd_vel(0.0, 0.005)  # 0.29 deg/s: below the 0.01 rad/s deadband
    imu_samples(sim, 2.0)
    assert all(gyro[2] == 0.0 for _, gyro, _ in imu_samples(sim, 1.0))
    sim.set_cmd_vel(0.0, 0.5)
    imu_samples(sim, 1.5)
    turning = np.mean([gyro[2] for _, gyro, _ in imu_samples(sim, 1.0)])
    assert turning == pytest.approx(0.5, rel=0.06)  # +-3 % sensitivity, as 2 sigma


def test_as5040_odometry_counts_whole_ticks(robot_sensors_sim):
    sim = robot_sensors_sim
    sim.reset(0.0, -5.0, 0.0)
    tick = 2.0 * math.pi / 1024
    angles = sim._encoder_angles()
    assert np.allclose(angles / tick, np.round(angles / tick))
    sim.set_cmd_vel(0.3, 0.0)
    sim.step(1000)
    travelled = sim.odom_travel[0]
    one_count = tick * sim.drive.wheel_radius
    assert travelled == pytest.approx(0.3 * 2.0 - 0.09, abs=0.05)  # 0.6 s of acceleration
    assert one_count == pytest.approx(0.000675, abs=1e-5)


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
