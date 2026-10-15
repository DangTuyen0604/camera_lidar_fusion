"""ROS-independent MuJoCo warehouse simulator for the OpenAMRobot AMR."""

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from . import mesh_tools, sdf_loader

HIDDEN_Z = -20.0


@dataclass
class DiffDriveConfig:
    """Same values as the former gz-sim-diff-drive-system plugin."""

    wheel_separation: float = 0.4075
    # Track used for drive and odometry. Kept at the hub spacing: arcs match
    # it (<1 cm over 3 m), while in-place spins scrub and turn ~1.8% further
    # than odometry reports, like a real AMR; AMCL corrects that yaw drift.
    effective_wheel_separation: float = 0.4075
    wheel_radius: float = 0.11
    max_linear_velocity: float = 1.0
    max_angular_velocity: float = 2.0
    max_linear_acceleration: float = 0.5
    max_angular_acceleration: float = 1.0


@dataclass
class LidarConfig:
    """Same values as the former gpu_lidar sensor in gazebo_control.xacro."""

    samples: int = 360
    min_angle: float = -3.14159
    max_angle: float = 3.14159
    range_min: float = 0.40
    range_max: float = 10.0
    noise_stddev: float = 0.001


@dataclass
class Lidar3dConfig:
    """
    Multi-layer LiDAR on the same mount, VLP-16-like.

    The planar scan stays the 2D sensor for AMCL and the costmaps; this one
    also sees the floor and anything below the 0.47 m scan plane, such as a
    0.18 m pallet.
    """

    channels: int = 16
    samples: int = 900
    min_elevation_deg: float = -15.0
    max_elevation_deg: float = 15.0
    range_min: float = 0.40
    range_max: float = 30.0
    noise_stddev: float = 0.01


@dataclass
class ImuConfig:
    """
    Error model of the 6-axis IMU; the defaults are an ideal sensor.

    A sample is the average over the period since the previous one, as a MEMS
    IMU's internal low-pass and decimation deliver it: the IMU site's mean
    specific force (dv/dt minus gravity, in the IMU frame) and mean angular
    rate. MuJoCo's own instantaneous accelerometer is dominated by contact
    chatter (+-12 m/s^2 spikes on this robot) that a real sensor's filter
    removes. Sample noise std is density * sqrt(rate / 2). The turn-on bias
    and the gyro scale error are drawn again at every reset().

    The firmware fields reproduce what the robot's microcontroller does to the
    chip's output before ROS sees it (linorobot2 IMUInterface): average the
    gyro over the first boot_calibration_samples at rest and subtract that,
    quantize to the configured full-scale range, and zero any gyro axis
    reading below gyro_deadband.
    """

    gyro_noise_density: float = 0.0       # [rad/s/sqrt(Hz)]
    accel_noise_density: float = 0.0      # [m/s^2/sqrt(Hz)]
    gyro_bias_std: float = 0.0            # turn-on bias per axis [rad/s]
    accel_bias_std: float = 0.0           # [m/s^2]
    gyro_bias_random_walk: float = 0.0    # [rad/s/sqrt(s)]
    accel_bias_random_walk: float = 0.0   # [m/s^2/sqrt(s)]
    gyro_scale_std: float = 0.0           # sensitivity error, fraction
    boot_calibration_samples: int = 0     # 0: the firmware subtracts no gyro offset
    gyro_lsb: float = 0.0                 # quantization step [rad/s]; 0: none
    accel_lsb: float = 0.0                # [m/s^2]
    gyro_deadband: float = 0.0            # |rate| below this reads 0 [rad/s]

    @classmethod
    def mpu6500(cls):
        """
        Model the OpenAMRobot's IMU: a TDK InvenSense MPU-6500 behind its firmware.

        Chip (PS-MPU-6500A-01, rev 1.3): rate noise 0.01 deg/s/sqrt(Hz),
        initial zero-rate output +-5 deg/s, sensitivity +-3 %; accelerometer
        300 ug/sqrt(Hz), zero-g +-60 mg. Tolerances are taken as 2 sigma. The
        bias random walks are an assumption: the zero-rate / zero-g drift of a
        ~2 K warm-up per hour (0.24 deg/s/K, 0.64 mg/K).

        Firmware (openamr-platform-fw on linorobot2_hardware, MPU9250 driver):
        +-250 deg/s and +-2 g ranges (131 LSB/(deg/s), 16384 LSB/g), gyro
        offset averaged over 40 samples at boot, 0.01 rad/s gyro deadband.
        """
        g = 9.80665
        return cls(gyro_noise_density=math.radians(0.01), accel_noise_density=300e-6 * g,
                   gyro_bias_std=math.radians(2.5), accel_bias_std=0.030 * g,
                   gyro_bias_random_walk=1.4e-4, accel_bias_random_walk=2.1e-4,
                   gyro_scale_std=0.015, boot_calibration_samples=40,
                   gyro_lsb=math.radians(1.0 / 131.0), accel_lsb=g / 16384.0,
                   gyro_deadband=0.01)


@dataclass
class EncoderConfig:
    """
    Wheel encoder and odometry calibration error; the defaults are exact.

    Odometry converts encoder counts with the nominal wheel radius and track,
    while the true ones differ by a per-wheel radius error and a track error
    drawn at every reset() (systematic odometry errors, UMBmark's E_d and E_b).
    """

    counts_per_revolution: int = 0        # 0: continuous angle
    radius_error_std: float = 0.0         # per wheel, fraction of the radius
    track_error_std: float = 0.0          # fraction of the wheel separation

    @classmethod
    def as5040(cls):
        """
        Model the OpenAMRobot's wheel encoders: AMS AS5040, 1024 counts per wheel revolution.

        COUNTS_PER_REV in openamr-platform-fw's lino_base_config.h. Wheel
        radii within ~0.2 % and the track within ~1 % are assumptions for an
        uncalibrated robot.
        """
        return cls(counts_per_revolution=1024, radius_error_std=0.002, track_error_std=0.01)


def yaw_to_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def quat_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WarehouseSim:
    """Warehouse physics, diff-drive control and sensors, without ROS."""

    def __init__(self, scene_xml, world_sdf=None, model_paths=(), meshdir=None,
                 entity_models=None, pool_size=4, spawn=(0.0, -4.0, 0.0),
                 drive=None, lidar=None, lidar_3d=None, seed=0, imu=None, encoder=None):
        self.drive = drive or DiffDriveConfig()
        self.lidar_cfg = lidar or LidarConfig()
        self.lidar_3d_cfg = lidar_3d
        self.imu_cfg = imu or ImuConfig()
        self.encoder_cfg = encoder or EncoderConfig()
        self.rng = np.random.default_rng(seed)

        spec = mujoco.MjSpec.from_file(str(scene_xml))
        if meshdir:
            spec.meshdir = str(Path(meshdir).resolve())
        self._resolve_meshes(spec, Path(scene_xml).parent)
        self.ground_z = 0.0
        if world_sdf:
            sdf_loader.add_world(spec, world_sdf, model_paths)
            self.ground_z = self._ground_height(spec)
        else:
            floor = spec.worldbody.add_geom()
            floor.type = mujoco.mjtGeom.mjGEOM_PLANE
            floor.size = [20, 20, 1]
            floor.rgba = [0.42, 0.45, 0.48, 1]
            floor.contype = floor.conaffinity = sdf_loader.FLOOR_BITS

        # MuJoCo cannot add bodies after compilation, so every model the
        # scenario may spawn gets a pool of hidden mocap bodies instead.
        self._pools = {}
        self._slot_geoms = {}
        for model_name, sdf_path in (entity_models or {}).items():
            model = sdf_loader.model_element(Path(sdf_path).read_text())
            slots = []
            for index in range(pool_size):
                body_name = f'entity_{model_name}_{index}'
                body = spec.worldbody.add_body(name=body_name, mocap=True)
                body.pos = [100.0 + 5 * len(self._pools), 5.0 * index, HIDDEN_Z]
                sdf_loader.add_model_geoms(body, model, prefix=f'{body_name}/', spec=spec,
                                           base_dir=Path(sdf_path).parent,
                                           model_paths=model_paths)
                slots.append(body_name)
            self._pools[model_name] = slots

        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.entities = {}  # scenario name -> (model_name, body_name)
        for slots in self._pools.values():
            for body_name in slots:
                body_id = self.model.body(body_name).id
                self._slot_geoms[body_name] = [
                    g for g in range(self.model.ngeom)
                    if self.model.geom_bodyid[g] == body_id
                    and self.model.geom_contype[g]]

        self.base_id = self.model.body('base_link').id
        self.lidar_site = self.model.site('lidar').id
        self.imu_site = self.model.site('imu').id
        joint = self.model.joint('base_free')
        self._qpos = joint.qposadr[0]
        self._qvel = joint.dofadr[0]
        self._left = self.model.joint('left_wheel_joint')
        self._right = self.model.joint('right_wheel_joint')
        self._left_act = self.model.actuator('left_wheel_motor').id
        self._right_act = self.model.actuator('right_wheel_motor').id
        angles = np.linspace(self.lidar_cfg.min_angle, self.lidar_cfg.max_angle,
                             self.lidar_cfg.samples)
        self.lidar_angles = angles
        self._ray_local = np.stack([np.cos(angles), np.sin(angles),
                                    np.zeros_like(angles)], axis=1)
        # Ray-cast the primitive collisions (group 3), not the detailed visuals:
        # returns then match the static map built from the same geometry.
        self._ray_group = np.zeros(6, dtype=np.uint8)
        self._ray_group[sdf_loader.COLLISION_GROUP] = 1
        if lidar_3d is not None:
            elevations = np.radians(np.linspace(
                lidar_3d.min_elevation_deg, lidar_3d.max_elevation_deg, lidar_3d.channels))
            azimuths = np.linspace(-math.pi, math.pi, lidar_3d.samples, endpoint=False)
            elevation, azimuth = np.meshgrid(elevations, azimuths, indexing='ij')
            self._ray_local_3d = np.stack([
                np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth),
                np.sin(elevation)], axis=-1).reshape(-1, 3)
            self.lidar_3d_rings = np.repeat(
                np.arange(lidar_3d.channels, dtype=np.uint16), lidar_3d.samples)

        self.cmd = np.zeros(2)       # requested (v, w)
        self.applied = np.zeros(2)   # acceleration-limited (v, w)
        self.reset(*spawn)

    @staticmethod
    def _resolve_meshes(spec, scene_dir):
        """Point meshes at meshdir and decimate those over MuJoCo's STL limit."""
        meshdir = (scene_dir / spec.meshdir).resolve()  # absolute: bypasses meshdir
        for mesh in spec.meshes:
            if not mesh.file:
                continue
            # Meshes from an <include>d file may already carry an absolute
            # path next to that file (e.g. an install/ symlink); only the
            # file name is meaningful.
            path = meshdir / Path(mesh.file).name
            reduced = mesh_tools.load_for_mujoco(path)
            if reduced is None:
                mesh.file = str(path)
            else:
                mesh.file = ''
                mesh.uservert = reduced[0].ravel()
                mesh.userface = reduced[1].ravel()

    @staticmethod
    def _ground_height(spec):
        planes = [g.pos[2] for g in spec.worldbody.geoms
                  if g.type == mujoco.mjtGeom.mjGEOM_PLANE]
        return float(max(planes)) if planes else 0.0

    # ------------------------------------------------------------------ robot
    def reset(self, x, y, yaw):
        mujoco.mj_resetData(self.model, self.data)
        q = self.data.qpos
        q[self._qpos:self._qpos + 3] = [x, y, self.ground_z + 0.066]
        q[self._qpos + 3:self._qpos + 7] = yaw_to_quat(yaw)
        for slots in self._pools.values():
            for body_name in slots:
                self._hide(body_name)
        self.entities.clear()
        self.cmd[:] = 0
        self.applied[:] = 0
        self.odom = np.zeros(3)  # wheel odometry x, y, yaw from spawn
        # Cumulative odometry travel (distance, yaw): twist = its difference
        # over a publishing period, as a drive controller derives it from counts.
        self.odom_travel = np.zeros(2)
        enc = self.encoder_cfg
        self.wheel_radius_error = self._normal(enc.radius_error_std, 2)
        self.track_error = float(self._normal(enc.track_error_std, 1)[0])
        mujoco.mj_forward(self.model, self.data)
        self._wheel_prev = self._encoder_angles()
        imu = self.imu_cfg
        self.gyro_bias = self._normal(imu.gyro_bias_std, 3)
        self.accel_bias = self._normal(imu.accel_bias_std, 3)
        self.gyro_scale = 1.0 + self._normal(imu.gyro_scale_std, 3)
        # Firmware boot calibration: the mean of n gyro samples at rest (one
        # per publishing period, 50 ms apart) is the turn-on bias plus their
        # averaged noise; it is subtracted from every later reading.
        self.gyro_offset = np.zeros(3)
        if imu.boot_calibration_samples > 0:
            sample_std = imu.gyro_noise_density * math.sqrt(1.0 / (2.0 * 0.05))
            self.gyro_offset = self.gyro_bias + self._normal(
                sample_std / math.sqrt(imu.boot_calibration_samples), 3)
        self._imu_prev = self._imu_kinematics()

    def set_cmd_vel(self, linear, angular):
        d = self.drive
        self.cmd[0] = float(np.clip(linear, -d.max_linear_velocity, d.max_linear_velocity))
        self.cmd[1] = float(np.clip(angular, -d.max_angular_velocity, d.max_angular_velocity))

    def step(self, n=1):
        dt = self.model.opt.timestep
        d = self.drive
        limits = np.array([d.max_linear_acceleration, d.max_angular_acceleration]) * dt
        for _ in range(n):
            self.applied += np.clip(self.cmd - self.applied, -limits, limits)
            v, w = self.applied
            half = d.effective_wheel_separation / 2
            self.data.ctrl[self._left_act] = (v - w * half) / d.wheel_radius
            self.data.ctrl[self._right_act] = (v + w * half) / d.wheel_radius
            mujoco.mj_step(self.model, self.data)
            self._integrate_odometry(dt)

    def _encoder_angles(self):
        """Wheel angles as the encoders report them (whole counts)."""
        angles = np.array([self.data.qpos[self._left.qposadr[0]],
                           self.data.qpos[self._right.qposadr[0]]])
        counts = self.encoder_cfg.counts_per_revolution
        if counts > 0:
            tick = 2.0 * math.pi / counts
            angles = np.floor(angles / tick) * tick
        return angles

    def _integrate_odometry(self, dt):
        d = self.drive
        wheels = self._encoder_angles()
        # Counts -> distance with the radius odometry assumes; the true wheel
        # differs from it by wheel_radius_error.
        radius = d.wheel_radius * (1.0 + self.wheel_radius_error)
        left, right = (wheels - self._wheel_prev) * radius
        self._wheel_prev = wheels
        ds = (left + right) / 2
        dyaw = (right - left) / (d.effective_wheel_separation * (1.0 + self.track_error))
        mid = self.odom[2] + dyaw / 2
        self.odom += [ds * math.cos(mid), ds * math.sin(mid), dyaw]
        self.odom[2] = math.atan2(math.sin(self.odom[2]), math.cos(self.odom[2]))
        self.odom_travel += [ds, dyaw]

    @property
    def time(self):
        return self.data.time

    def robot_pose(self):
        """Ground-truth (x, y, yaw) of base_link in the world frame."""
        q = self.data.qpos
        x, y = q[self._qpos:self._qpos + 2]
        return float(x), float(y), quat_to_yaw(q[self._qpos + 3:self._qpos + 7])

    def wheel_state(self):
        position = [self.data.qpos[self._left.qposadr[0]],
                    self.data.qpos[self._right.qposadr[0]]]
        velocity = [self.data.qvel[self._left.dofadr[0]],
                    self.data.qvel[self._right.dofadr[0]]]
        return position, velocity

    def imu(self):
        """Return MuJoCo's instantaneous ideal (orientation wxyz, gyro, accelerometer)."""
        sensor = self.data.sensor
        return (sensor('imu_quat').data.copy(), sensor('imu_gyro').data.copy(),
                sensor('imu_accel').data.copy())

    def _imu_kinematics(self):
        velocity = np.zeros(6)  # world-aligned [angular, linear] of the IMU site
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE,
                                 self.imu_site, velocity, 0)
        return (self.data.time, velocity[3:].copy(),
                self.data.site_xmat[self.imu_site].reshape(3, 3).copy())

    def imu_sample(self):
        """
        Return the IMU sample for the period since the previous call.

        (orientation wxyz, angular velocity, specific force), the last two in
        the IMU frame with ImuConfig's errors applied; the orientation is the
        true one (a 6-axis IMU outputs none).
        """
        now, velocity, rotation = self._imu_kinematics()
        before, velocity_before, rotation_before = self._imu_prev
        dt = now - before
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rotation.ravel())
        if dt <= 0.0:
            return (quat, self._firmware_gyro(self.gyro_bias.copy()),
                    self._quantize(rotation.T @ -self.model.opt.gravity + self.accel_bias,
                                   self.imu_cfg.accel_lsb))
        self._imu_prev = (now, velocity, rotation)
        specific_force = rotation.T @ ((velocity - velocity_before) / dt - self.model.opt.gravity)
        relative = np.zeros(4)
        mujoco.mju_mat2Quat(relative, (rotation_before.T @ rotation).ravel())
        rate = np.zeros(3)
        mujoco.mju_quat2Vel(rate, relative, dt)

        cfg = self.imu_cfg
        self.gyro_bias += self._normal(cfg.gyro_bias_random_walk * math.sqrt(dt), 3)
        self.accel_bias += self._normal(cfg.accel_bias_random_walk * math.sqrt(dt), 3)
        sample_scale = math.sqrt(1.0 / (2.0 * dt))
        gyro = (self.gyro_scale * rate + self.gyro_bias +
                self._normal(cfg.gyro_noise_density * sample_scale, 3))
        accel = (specific_force + self.accel_bias +
                 self._normal(cfg.accel_noise_density * sample_scale, 3))
        return quat, self._firmware_gyro(gyro), self._quantize(accel, cfg.accel_lsb)

    def _firmware_gyro(self, gyro):
        """Quantize, subtract the boot offset and apply the deadband, as the firmware does."""
        gyro = self._quantize(gyro, self.imu_cfg.gyro_lsb) - self.gyro_offset
        gyro[np.abs(gyro) < self.imu_cfg.gyro_deadband] = 0.0
        return gyro

    @staticmethod
    def _quantize(values, step):
        return np.round(values / step) * step if step > 0.0 else values

    def _normal(self, std, size):
        """Zero-mean noise; draws nothing for std 0, so ideal sensors keep the RNG stream."""
        return self.rng.normal(0.0, std, size) if std > 0.0 else np.zeros(size)

    # ----------------------------------------------------------------- sensors
    def scan(self):
        """360-beam planar scan from lidar_link; out-of-range beams are inf."""
        cfg = self.lidar_cfg
        origin = self.data.site_xpos[self.lidar_site].copy()
        rotation = self.data.site_xmat[self.lidar_site].reshape(3, 3)
        directions = (self._ray_local @ rotation.T).ravel()
        distances = np.full(cfg.samples, -1.0)
        geom_ids = np.full(cfg.samples, -1, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, origin, directions,
                           self._ray_group, 1, self.base_id, geom_ids, distances,
                           None, cfg.samples, cfg.range_max)
        ranges = distances + self.rng.normal(0.0, cfg.noise_stddev, cfg.samples)
        invalid = (geom_ids < 0) | (ranges < cfg.range_min) | (ranges > cfg.range_max)
        ranges[invalid] = np.inf
        return ranges.astype(np.float32)

    def scan_3d(self):
        """Return (points in lidar_link [N x 3] float32, ring [N] uint16) of valid returns."""
        cfg = self.lidar_3d_cfg
        if cfg is None:
            raise RuntimeError('WarehouseSim was created without a 3D LiDAR')
        count = len(self._ray_local_3d)
        origin = self.data.site_xpos[self.lidar_site].copy()
        rotation = self.data.site_xmat[self.lidar_site].reshape(3, 3)
        directions = (self._ray_local_3d @ rotation.T).ravel()
        distances = np.full(count, -1.0)
        geom_ids = np.full(count, -1, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, origin, directions,
                           self._ray_group, 1, self.base_id, geom_ids, distances,
                           None, count, cfg.range_max)
        ranges = distances + self.rng.normal(0.0, cfg.noise_stddev, count)
        valid = (geom_ids >= 0) & (ranges >= cfg.range_min) & (ranges <= cfg.range_max)
        points = self._ray_local_3d[valid] * ranges[valid, None]
        return points.astype(np.float32), self.lidar_3d_rings[valid]

    # ---------------------------------------------------------------- entities
    def entity_models(self):
        return list(self._pools)

    def spawn_entity(self, name, model_name, pos, quat):
        if name in self.entities:
            raise ValueError(f'entity {name!r} already exists')
        if model_name not in self._pools:
            raise KeyError(f'no entity pool for model {model_name!r}')
        used = {body for _, body in self.entities.values()}
        free = [body for body in self._pools[model_name] if body not in used]
        if not free:
            raise RuntimeError(f'entity pool for {model_name!r} is exhausted')
        self.entities[name] = (model_name, free[0])
        self.set_entity_pose(name, pos, quat)

    def set_entity_pose(self, name, pos, quat):
        _, body_name = self.entities[name]
        mocap = self.model.body(body_name).mocapid[0]
        pos = np.asarray(pos, dtype=float).copy()
        pos[2] += self.ground_z
        self.data.mocap_pos[mocap] = pos
        self.data.mocap_quat[mocap] = quat
        # Cargo carried on the robot is posed inside the robot footprint by
        # the scenario (attach_cargo).  A kinematic mocap body there would
        # pin the robot, so carried items are sensor-visible but contact-free.
        x, y, _ = self.robot_pose()
        carried = pos[2] - self.ground_z > 0.3 and math.hypot(pos[0] - x, pos[1] - y) < 0.6
        for geom in self._slot_geoms[body_name]:
            self.model.geom_contype[geom] = 0 if carried else 1
            self.model.geom_conaffinity[geom] = 0 if carried else 1

    def entity_pose(self, name):
        _, body_name = self.entities[name]
        mocap = self.model.body(body_name).mocapid[0]
        pos = self.data.mocap_pos[mocap].copy()
        pos[2] -= self.ground_z
        return pos, self.data.mocap_quat[mocap].copy()

    def delete_entity(self, name):
        _, body_name = self.entities.pop(name)
        self._hide(body_name)

    def _hide(self, body_name):
        mocap = self.model.body(body_name).mocapid[0]
        self.data.mocap_pos[mocap, 2] = HIDDEN_Z
