"""
ROS 2 front end of the MuJoCo warehouse simulator.

ROS contract (identical to the former Gazebo server + ros_gz_bridge):
  subscribes /cmd_vel (geometry_msgs/Twist)
  publishes  /clock /odom /tf(odom->base_footprint) /joint_states /imu /scan
             /lidar/points /camera/image_raw /camera/camera_info
             /ground_truth/odom (extra: world-frame truth for benchmarks)
  services   <ns>/spawn_entity <ns>/set_entity_state <ns>/delete_entity
             (simulation_interfaces, as used by warehouse_scenario_runner)
"""

import math
from pathlib import Path
import threading
import time
from urllib.parse import unquote, urlparse

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped, Twist
import mujoco
import mujoco.viewer
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import DeleteEntity, SetEntityState, SpawnEntity
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from . import sdf_loader
from .simulator import quat_to_yaw, WarehouseSim, yaw_to_quat


# TF root of the URDF: base_footprint sits on the floor, 0.0635 m below
# base_link (base_footprint_joint in robo_urdf.urdf.xacro).
BASE_FRAME = 'base_footprint'
FOOTPRINT_HEIGHT = 0.0635


def _share(package):
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError:
        return None


def _default_paths():
    """Resolve scene/mesh paths for both installed and source-tree use."""
    source_root = Path(__file__).resolve().parents[1]
    own_share = _share('warehouse_mujoco')
    description = _share('openamrobot_description')
    scene = (own_share / 'mjcf' / 'scene.xml') if own_share else source_root / 'mjcf' / 'scene.xml'
    meshdir = (description / 'meshes' / 'visual') if description else None
    return str(scene), str(meshdir) if meshdir else ''


def _stamp(seconds):
    # Split after rounding: 1.9999999999998 must become 2 s, not 1 s + 0 ns,
    # or /clock jumps back a second and every TF buffer is cleared.
    sec, nanosec = divmod(int(round(seconds * 1e9)), 1000000000)
    return TimeMsg(sec=sec, nanosec=nanosec)


class MujocoSimNode(Node):
    """Owns the simulator; the physics loop runs in `run()` on the main thread."""

    def __init__(self):
        super().__init__('mujoco_sim')
        scene, meshdir = _default_paths()
        p = self.declare_parameter
        scene = p('scene_xml', scene).value
        meshdir = p('meshdir', meshdir).value
        world = p('world', '').value
        model_paths = [path for path in p('model_paths', '').value.split(':') if path]
        entity_names = [
            n for n in p('entity_models', ['worker', 'cargo_box', 'pallet']).value if n]
        pool_size = int(p('entity_pool_size', 4).value)
        spawn = (float(p('spawn_x', 0.0).value), float(p('spawn_y', -4.0).value),
                 float(p('spawn_yaw', 0.0).value))
        self.gui = bool(p('gui', False).value)
        self.real_time_factor = float(p('real_time_factor', 1.0).value)
        self.cmd_vel_timeout = float(p('cmd_vel_timeout', 0.0).value)
        self.enable_camera = bool(p('enable_camera', True).value)
        self.camera_size = (int(p('camera_width', 640).value), int(p('camera_height', 480).value))
        self.camera_hfov = float(p('camera_horizontal_fov', 1.047).value)
        service_ns = p('service_namespace', '/mujoco').value.rstrip('/')
        rates = {name: float(p(f'{name}_rate', default).value) for name, default in (
            ('clock', 250.0), ('odom', 50.0), ('joint_states', 50.0), ('imu', 100.0),
            ('scan', 10.0), ('camera', 12.0))}

        entity_models = {}
        for name in entity_names:
            try:
                entity_models[name] = sdf_loader.find_model_file(f'model://{name}', model_paths)
            except FileNotFoundError:
                self.get_logger().warn(f'No SDF for spawnable model {name!r}; skipped')
        started = time.monotonic()
        self.sim = WarehouseSim(scene, world or None, model_paths, meshdir or None,
                                entity_models, pool_size, spawn)
        self.get_logger().info(
            f'MuJoCo {mujoco.__version__} loaded {Path(world).name if world else "empty world"} '
            f'in {time.monotonic() - started:.1f}s; robot at {spawn}; '
            f'spawnable: {self.sim.entity_models()} x{pool_size}')

        self.lock = threading.Lock()
        self.last_cmd_time = None
        self.periods = {name: 1.0 / rate for name, rate in rates.items() if rate > 0}
        self.next_due = {name: 0.0 for name in self.periods}

        group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel, 10, callback_group=group)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/lidar/points', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.create_service(SpawnEntity, f'{service_ns}/spawn_entity', self._spawn,
                            callback_group=group)
        self.create_service(SetEntityState, f'{service_ns}/set_entity_state', self._set_state,
                            callback_group=group)
        self.create_service(DeleteEntity, f'{service_ns}/delete_entity', self._delete,
                            callback_group=group)

    # --------------------------------------------------------------- callbacks
    def _cmd_vel(self, msg):
        with self.lock:
            self.sim.set_cmd_vel(msg.linear.x, msg.angular.z)
            self.last_cmd_time = time.monotonic()

    @staticmethod
    def _pose_arrays(pose):
        q = pose.orientation
        return ([pose.position.x, pose.position.y, pose.position.z], [q.w, q.x, q.y, q.z])

    @staticmethod
    def _model_name(request):
        """Model name from resource_string, file:// URI or model:// URI."""
        if request.resource_string:
            return sdf_loader.model_element(request.resource_string).get('name')
        if request.uri.startswith('file://'):
            text = Path(unquote(urlparse(request.uri).path)).read_text()
            return sdf_loader.model_element(text).get('name')
        return request.uri.replace('model://', '').strip('/')

    def _spawn(self, request, response):
        name = request.name
        try:
            model = self._model_name(request)
        except Exception as error:  # Malformed SDF is reported to the caller.
            response.result.result = SpawnEntity.Response.RESOURCE_PARSE_ERROR
            response.result.error_message = str(error)
            return response
        with self.lock:
            if not name:
                response.result.result = SpawnEntity.Response.NAME_INVALID
                response.result.error_message = 'entity name is empty'
            elif name in self.sim.entities:
                response.result.result = SpawnEntity.Response.NAME_NOT_UNIQUE
                response.result.error_message = f'entity {name!r} already exists'
            elif model not in self.sim.entity_models():
                response.result.result = SpawnEntity.Response.UNSUPPORTED_ASSETS
                response.result.error_message = (
                    f'model {model!r} has no MuJoCo entity pool; '
                    f'spawnable: {self.sim.entity_models()}')
            else:
                try:
                    self.sim.spawn_entity(
                        name, model, *self._pose_arrays(request.initial_pose.pose))
                    response.result.result = Result.RESULT_OK
                    response.entity_name = name
                except RuntimeError as error:
                    response.result.result = Result.RESULT_OPERATION_FAILED
                    response.result.error_message = str(error)
        return response

    def _set_state(self, request, response):
        with self.lock:
            if request.entity not in self.sim.entities:
                response.result.result = Result.RESULT_NOT_FOUND
                response.result.error_message = f'unknown entity {request.entity!r}'
            else:
                self.sim.set_entity_pose(request.entity, *self._pose_arrays(request.state.pose))
                response.result.result = Result.RESULT_OK
        return response

    def _delete(self, request, response):
        with self.lock:
            if request.entity not in self.sim.entities:
                response.result.result = Result.RESULT_NOT_FOUND
                response.result.error_message = f'unknown entity {request.entity!r}'
            else:
                self.sim.delete_entity(request.entity)
                response.result.result = Result.RESULT_OK
        return response

    # ---------------------------------------------------------------- publish
    def _publish_clock(self, stamp):
        self.clock_pub.publish(Clock(clock=stamp))

    def _publish_odom(self, stamp):
        sim = self.sim
        x, y, yaw = sim.odom
        _, wheel_velocity = sim.wheel_state()
        radius, separation = sim.drive.wheel_radius, sim.drive.effective_wheel_separation
        linear = radius * (wheel_velocity[0] + wheel_velocity[1]) / 2
        angular = radius * (wheel_velocity[1] - wheel_velocity[0]) / separation
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(yaw)
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.z = float(q[0]), float(q[3])
        odom.twist.twist.linear.x = float(linear)
        odom.twist.twist.angular.z = float(angular)
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 1e-4
            odom.twist.covariance[index] = 1e-4
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[transform]))

        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'world'
        truth.child_frame_id = BASE_FRAME
        qpos = sim.data.qpos
        truth.pose.pose.position.x, truth.pose.pose.position.y = float(qpos[0]), float(qpos[1])
        truth.pose.pose.position.z = float(qpos[2] - sim.ground_z - FOOTPRINT_HEIGHT)
        o = truth.pose.pose.orientation
        o.w, o.x, o.y, o.z = (float(v) for v in qpos[3:7])
        yaw_true = quat_to_yaw(qpos[3:7])
        vx, vy = sim.data.qvel[0:2]
        truth.twist.twist.linear.x = float(vx * math.cos(yaw_true) + vy * math.sin(yaw_true))
        truth.twist.twist.linear.y = float(-vx * math.sin(yaw_true) + vy * math.cos(yaw_true))
        truth.twist.twist.angular.z = float(sim.data.qvel[5])
        self.truth_pub.publish(truth)

    def _publish_joint_states(self, stamp):
        position, velocity = self.sim.wheel_state()
        self.joint_pub.publish(JointState(
            header=self._header(stamp, ''),
            name=['left_wheel_joint', 'right_wheel_joint'],
            position=[float(v) for v in position], velocity=[float(v) for v in velocity]))

    def _publish_imu(self, stamp):
        quat, gyro, accel = self.sim.imu()
        imu = Imu(header=self._header(stamp, 'imu_link'))
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (
            float(v) for v in quat)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
            float(v) for v in gyro)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            float(v) for v in accel)
        self.imu_pub.publish(imu)

    def _publish_scan(self, stamp):
        cfg = self.sim.lidar_cfg
        ranges = self.sim.scan()
        scan = LaserScan(header=self._header(stamp, 'lidar_link'))
        scan.angle_min, scan.angle_max = cfg.min_angle, cfg.max_angle
        scan.angle_increment = (cfg.max_angle - cfg.min_angle) / (cfg.samples - 1)
        scan.scan_time = self.periods['scan']
        scan.range_min, scan.range_max = cfg.range_min, cfg.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        finite = np.isfinite(ranges)
        angles = self.sim.lidar_angles[finite]
        points = np.zeros((int(finite.sum()), 4), dtype=np.float32)
        points[:, 0] = ranges[finite] * np.cos(angles)
        points[:, 1] = ranges[finite] * np.sin(angles)
        points[:, 3] = 100.0
        cloud = PointCloud2(header=scan.header, height=1, width=len(points),
                            is_bigendian=False, point_step=16, row_step=16 * len(points),
                            is_dense=True, data=points.tobytes())
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        self.cloud_pub.publish(cloud)

    def _publish_camera(self, stamp, renderer):
        renderer.update_scene(self.sim.data, camera='front_camera')
        pixels = renderer.render()
        width, height = self.camera_size
        header = self._header(stamp, 'camera_optical_frame')
        self.image_pub.publish(Image(header=header, height=height, width=width,
                                     encoding='rgb8', is_bigendian=0, step=3 * width,
                                     data=pixels.tobytes()))
        focal = (width / 2) / math.tan(self.camera_hfov / 2)
        info = CameraInfo(header=header, height=height, width=width,
                          distortion_model='plumb_bob', d=[0.0] * 5)
        info.k = [focal, 0.0, width / 2, 0.0, focal, height / 2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, width / 2, 0.0, 0.0, focal, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    @staticmethod
    def _header(stamp, frame):
        return Header(stamp=stamp, frame_id=frame)

    # ------------------------------------------------------------------- loop
    def _due(self, name, now):
        """Fixed-phase scheduler: rates stay exact despite the 4 ms tick."""
        if now < self.next_due[name]:
            return False
        self.next_due[name] += self.periods[name]
        if self.next_due[name] <= now:  # Fell behind: skip, do not burst.
            self.next_due[name] = now + self.periods[name]
        return True

    def run(self):
        """Step physics in (scaled) real time and publish sensors when due."""
        sim = self.sim
        viewer = None
        if self.gui:
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data,
                                                  show_left_ui=False, show_right_ui=False)
            with viewer.lock():
                viewer.cam.lookat[:] = [0.0, 0.0, 0.0]
                viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 20.0, -50.0, 90.0
        renderer = None
        if self.enable_camera and 'camera' in self.periods:
            width, height = self.camera_size
            sim.model.vis.global_.offwidth = max(sim.model.vis.global_.offwidth, width)
            sim.model.vis.global_.offheight = max(sim.model.vis.global_.offheight, height)
            # znear is a fraction of stat.extent, which the parked entity pool
            # (x >= 100 m) inflates to metres: the camera then clips away any
            # dock or box close in front of it. Pin the near plane to 2 cm.
            sim.model.vis.map.znear = 0.02 / sim.model.stat.extent
            renderer = mujoco.Renderer(sim.model, height, width)

        steps_per_tick = max(1, round(0.004 / sim.model.opt.timestep))
        wall_start, sim_start = time.monotonic(), sim.time
        next_view = 0.0
        publishers = {
            'clock': self._publish_clock, 'odom': self._publish_odom,
            'joint_states': self._publish_joint_states, 'imu': self._publish_imu,
            'scan': self._publish_scan,
        }
        try:
            while rclpy.ok():
                with self.lock:
                    if (self.cmd_vel_timeout > 0 and self.last_cmd_time is not None
                            and time.monotonic() - self.last_cmd_time > self.cmd_vel_timeout):
                        sim.set_cmd_vel(0.0, 0.0)
                        self.last_cmd_time = None
                    sim.step(steps_per_tick)
                    now = sim.time
                    stamp = _stamp(now)
                    for name, publish in publishers.items():
                        if name in self.periods and self._due(name, now):
                            publish(stamp)
                    if renderer is not None and self._due('camera', now):
                        self._publish_camera(stamp, renderer)
                    if (viewer is not None and viewer.is_running()
                            and time.monotonic() >= next_view):
                        next_view = time.monotonic() + 1 / 60
                        viewer.sync()
                if self.real_time_factor > 0:
                    target = wall_start + (now - sim_start) / self.real_time_factor
                    delay = target - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    elif delay < -1.0:  # Cannot keep up: do not try to catch up forever.
                        wall_start, sim_start = time.monotonic(), now
        finally:
            if renderer is not None:
                renderer.close()
            if viewer is not None:
                viewer.close()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
