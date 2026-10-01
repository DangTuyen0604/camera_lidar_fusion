"""Stop and wait for moving obstacles detected in the LiDAR scan.

Velocity gate in the Nav2 chain between the velocity smoother and Collision
Monitor (cmd_vel_smoothed -> cmd_vel_yield), so navigation, recoveries and
docking are all covered while Collision Monitor stays the last safety layer.
It can only zero commands, never create or increase them.

Moving obstacles come from motion_tracker: unmapped scan clusters tracked
in odom; anything static (walls, racks, a parked pallet) never triggers it.
"""

from collections import deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from warehouse_mission_manager.motion_tracker import MotionTracker, TrackerConfig, clusters
from warehouse_mission_manager.yield_gate import YieldConfig, YieldGate


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose2d(x, y, yaw):
    """(R, t) of a planar pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]]), np.array([x, y])


def planar(transform):
    """(R, t) of a TransformStamped projected onto the ground plane."""
    t = transform.transform.translation
    return pose2d(t.x, t.y, yaw_of(transform.transform.rotation))


def compose(a, b):
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def inverse(a):
    return a[0].T, -a[0].T @ a[1]


def interpolate(history, t, max_extrapolation=0.1):
    """(x, y, yaw) at time t from a time-sorted [(t, x, y, yaw)], or None."""
    if not history or t < history[0][0]:
        return None
    if t >= history[-1][0]:
        return history[-1][1:] if t - history[-1][0] <= max_extrapolation else None
    for (t0, *a), (t1, *b) in zip(history, list(history)[1:]):
        if t0 <= t <= t1:
            r = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
            return (a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), a[2] + r * dyaw)
    return None


class DynamicYieldNode(Node):

    def __init__(self):
        super().__init__('dynamic_yield')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.map_frame = p('map_frame', 'map').value
        self.static_margin = float(p('static_margin', 0.2).value)
        self.max_range = float(p('max_range', 6.0).value)
        self.enabled = bool(p('enabled', True).value)
        self.tracker = MotionTracker(TrackerConfig(
            moving_speed=float(p('moving_speed', 0.35).value),
            memory_time=float(p('memory_time', 3.0).value)))
        self.gate = YieldGate(YieldConfig(
            stop_distance=float(p('stop_distance', 1.0).value),
            resume_distance=float(p('resume_distance', 1.5).value),
            clear_hold=float(p('clear_hold', 1.0).value),
            max_wait=float(p('max_wait', 10.0).value)))
        self.scan_timeout = float(p('scan_timeout', 0.5).value)

        # TF only for the fixed LiDAR mount and the slow map->odom drift.  The
        # robot pose at each scan comes from /odom, interpolated at the scan
        # stamp: Python TF lags ~1 s behind under load, and a pose that lags
        # makes static objects look like they move.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.odom_history = deque(maxlen=150)  # 3 s at 50 Hz
        self.lidar_mount = None
        self.static_distance = None
        self.map_info = None
        self.dynamic_odom = []     # (x, y) of moving tracks in odom
        self.scan_stamp = None
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value,
                                 self._map, map_qos)
        self.create_subscription(Odometry, p('odom_topic', '/odom').value,
                                 self._odom, 50)
        self.create_subscription(LaserScan, p('scan_topic', '/scan_filtered').value,
                                 self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_in', self._command, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/tracks', 10)
        cfg = self.gate.config
        self.get_logger().info(
            f'Dynamic yield {"enabled" if self.enabled else "disabled"}: obstacles faster '
            f'than {self.tracker.config.moving_speed} m/s; stop at {cfg.stop_distance} m, '
            f'resume beyond {cfg.resume_distance} m, wait at most {cfg.max_wait} s')

    # ------------------------------------------------------------- inputs
    def _map(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.static_distance = distance_transform_edt(grid < 50) * msg.info.resolution
        self.map_info = msg.info

    def _on_static_map(self, points):
        info = self.map_info
        i = ((points[:, 0] - info.origin.position.x) / info.resolution).astype(int)
        j = ((points[:, 1] - info.origin.position.y) / info.resolution).astype(int)
        inside = (i >= 0) & (i < info.width) & (j >= 0) & (j < info.height)
        near = np.ones(len(points), bool)  # outside the map counts as static
        near[inside] = self.static_distance[j[inside], i[inside]] < self.static_margin
        return near

    def _odom(self, msg):
        pose = msg.pose.pose
        self.odom_history.append((Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
                                  pose.position.x, pose.position.y,
                                  yaw_of(pose.orientation)))

    def _scan(self, scan):
        if self.static_distance is None:
            return
        now = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        robot = interpolate(self.odom_history, now)
        if robot is None:
            self.get_logger().warn('No odometry at scan time', throttle_duration_sec=5.0)
            return
        ranges = np.asarray(scan.ranges, float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges <= self.max_range)
        local = np.c_[ranges[valid] * np.cos(angles[valid]),
                      ranges[valid] * np.sin(angles[valid])]
        try:
            if self.lidar_mount is None:
                self.lidar_mount = planar(self.tf_buffer.lookup_transform(
                    self.base_frame, scan.header.frame_id, Time()))
            to_odom = compose(pose2d(*robot), self.lidar_mount)
            # AMCL publishes map->odom about a second late; it drifts slowly,
            # so the latest one is good enough to mask the static map.
            odom_to_map = planar(self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time()))
        except TransformException as error:
            self.get_logger().warn(f'No scan pose: {error}', throttle_duration_sec=5.0)
            return
        odom_all = local @ to_odom[0].T + to_odom[1]
        unmapped = ~self._on_static_map(odom_all @ odom_to_map[0].T + odom_to_map[1])
        odom_points = odom_all[unmapped]
        self.tracker.update(now, clusters(odom_points, self.tracker.config))
        dynamic = self.tracker.dynamic(now)
        self.dynamic_odom = [t.position.copy() for t in dynamic]
        if dynamic:
            nearest = min(math.hypot(*(t.position - to_odom[1])) for t in dynamic)
            fastest = max(t.speed for t in dynamic)
            self.get_logger().info(
                f'[TRACK] {len(dynamic)} moving obstacle(s), nearest {nearest:.2f} m, '
                f'fastest {fastest:.2f} m/s', throttle_duration_sec=1.0)
        self.scan_stamp = self.get_clock().now()
        self._publish_markers(scan.header.stamp, now)

    def _obstacles_in_base(self):
        if not self.dynamic_odom:
            return []
        rotation, translation = inverse(pose2d(*self.odom_history[-1][1:]))
        return [tuple(rotation @ p + translation) for p in self.dynamic_odom]

    # ------------------------------------------------------------ gating
    def _command(self, cmd):
        now = self.get_clock().now()
        fresh = (self.scan_stamp is not None and
                 (now - self.scan_stamp).nanoseconds * 1e-9 <= self.scan_timeout)
        try:
            obstacles = self._obstacles_in_base() if fresh else []
        except TransformException:
            obstacles = []
        before = self.gate.state
        allowed = self.gate.update(now.nanoseconds * 1e-9, obstacles,
                                   reversing=cmd.linear.x < 0.0)
        if self.gate.state != before:
            nearest = min((math.hypot(x, y) for x, y in obstacles), default=None)
            self.get_logger().info(
                f'[YIELD] {before} -> {self.gate.state} (nearest moving obstacle: '
                f'{"none" if nearest is None else f"{nearest:.2f} m"})')
            self.state_pub.publish(String(data=self.gate.state))
        self.cmd_pub.publish(cmd if allowed or not self.enabled else Twist())

    def _publish_markers(self, stamp, now):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for track in self.tracker.tracks:
            moving = track.recently_moving(now, self.tracker.config.memory_time)
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.odom_frame, stamp
            marker.ns, marker.id, marker.type = 'tracks', track.id, Marker.CYLINDER
            marker.pose.position.x, marker.pose.position.y = map(float, track.position)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.4
            marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b = (1.0, 0.1, 0.1) if moving else (
                0.5, 0.5, 0.5)
            marker.color.a = 0.8 if moving else 0.3
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicYieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
