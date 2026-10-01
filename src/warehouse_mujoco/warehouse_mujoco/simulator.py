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


def yaw_to_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def quat_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WarehouseSim:
    """Warehouse physics, diff-drive control and sensors, without ROS."""

    def __init__(self, scene_xml, world_sdf=None, model_paths=(), meshdir=None,
                 entity_models=None, pool_size=4, spawn=(0.0, -4.0, 0.0),
                 drive=None, lidar=None, seed=0):
        self.drive = drive or DiffDriveConfig()
        self.lidar_cfg = lidar or LidarConfig()
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
        self.odom_twist = np.zeros(2)
        self._wheel_prev = np.array([self.data.qpos[self._left.qposadr[0]],
                                     self.data.qpos[self._right.qposadr[0]]])
        mujoco.mj_forward(self.model, self.data)

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

    def _integrate_odometry(self, dt):
        d = self.drive
        wheels = np.array([self.data.qpos[self._left.qposadr[0]],
                           self.data.qpos[self._right.qposadr[0]]])
        left, right = (wheels - self._wheel_prev) * d.wheel_radius
        self._wheel_prev = wheels
        ds = (left + right) / 2
        dyaw = (right - left) / d.effective_wheel_separation
        mid = self.odom[2] + dyaw / 2
        self.odom += [ds * math.cos(mid), ds * math.sin(mid), dyaw]
        self.odom[2] = math.atan2(math.sin(self.odom[2]), math.cos(self.odom[2]))
        self.odom_twist[:] = [ds / dt, dyaw / dt]

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
        """Return (orientation wxyz, angular velocity, linear acceleration)."""
        sensor = self.data.sensor
        return (sensor('imu_quat').data.copy(), sensor('imu_gyro').data.copy(),
                sensor('imu_accel').data.copy())

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
