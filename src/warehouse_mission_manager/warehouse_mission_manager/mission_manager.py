"""M01-M04 action-driven warehouse mission state machine."""

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .cargo_manager import CargoManager
from .docking_controller import DockingController, pose_stamped
from .mission_state import MissionState


def navigation_succeeded(response):
    """Accept only a completed Nav2 goal, never a canceled/aborted result."""
    return (response.status == GoalStatus.STATUS_SUCCEEDED and
            response.result.error_code == 0)


class MissionManager(Node):
    def __init__(self):
        super().__init__('warehouse_mission_manager')
        share = Path(get_package_share_directory('warehouse_mission_manager'))
        stations_file = Path(self.declare_parameter(
            'stations_file', str(share / 'config' / 'stations.yaml')).value)
        missions_file = Path(self.declare_parameter(
            'missions_file', str(share / 'config' / 'missions.yaml')).value)
        self.stations = yaml.safe_load(stations_file.read_text())['stations']
        self.missions = yaml.safe_load(missions_file.read_text())['missions']
        self.state = MissionState.IDLE
        self.started = bool(self.declare_parameter('autostart', True).value)
        # false: stop at each station's staging pose (facing it, 0.70 m out),
        # pause and continue. true: run the full DockRobot/UndockRobot cycle.
        self.dock_at_stations = bool(
            self.declare_parameter('dock_at_stations', True).value)
        self.mission_index = 0
        self.odom = None
        self.collision = False
        self.deadline = None
        self.dock_validation_deadline = None
        self.cargo = CargoManager()
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigator_state = self.create_client(
            GetState, '/bt_navigator/get_state')
        self.navigator_state_future = None
        self.navigator_active = False
        self.waiting_for_navigation_logged = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.docking = DockingController(self)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.cargo_pub = self.create_publisher(String, '/mission/cargo', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state', self._collision, 10)
        self.create_service(Trigger, '/mission/start', self._start_mission)
        self.timer = self.create_timer(0.20, self._tick)
        self._publish_state()

    @property
    def mission(self):
        return self.missions[self.mission_index]

    def _odom(self, msg):
        self.odom = msg

    def _collision(self, msg):
        self.collision = msg.action_type == CollisionMonitorState.STOP

    def _start_mission(self, request, response):
        del request
        if self.state != MissionState.IDLE or self.started:
            response.success = False
            response.message = 'mission already started'
            return response
        self.started = True
        response.success = True
        response.message = 'M01-M04 mission sequence accepted'
        return response

    def _transition(self, state):
        self.state = state
        self.get_logger().info(f'{self.mission["id"]}: {state.value}')
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        cargo = String()
        cargo.data = self.cargo.cargo_id or ''
        self.cargo_pub.publish(cargo)

    def _navigation_ready(self):
        """Return true only after bt_navigator is lifecycle-active.

        An action server may be discoverable while its lifecycle node is still
        inactive.  Waiting for the lifecycle state avoids sending startup goals
        which Nav2 must reject.
        """
        if self.navigator_active:
            return True
        if self.navigator_state_future is not None:
            if not self.navigator_state_future.done():
                return False
            try:
                response = self.navigator_state_future.result()
            except Exception as error:
                self.get_logger().warn(
                    f'[MISSION] Could not read bt_navigator state: {error!r}')
            else:
                self.navigator_active = (
                    response.current_state.id == State.PRIMARY_STATE_ACTIVE)
            self.navigator_state_future = None
            if self.navigator_active:
                self.get_logger().info('[MISSION] NavigateToPose server ready')
                return True
        if self.navigator_state.service_is_ready():
            self.navigator_state_future = self.navigator_state.call_async(
                GetState.Request())
        if not self.waiting_for_navigation_logged:
            self.get_logger().info('[MISSION] Waiting for NavigateToPose server')
            self.waiting_for_navigation_logged = True
        return False

    def _navigate(self, station_name, callback, offset=0.0):
        if not self.navigation.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('[MISSION] Waiting for NavigateToPose server')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped(self, self.stations[station_name], offset)
        self.get_logger().info(
            f'[MISSION] Sending goal {self.mission["id"]} to {station_name}')
        future = self.navigation.send_goal_async(goal)

        def accepted(done):
            try:
                handle = done.result()
            except Exception as error:
                self.get_logger().error(
                    f'[MISSION] Goal request failed: {error!r}')
                callback(False)
                return
            if not handle.accepted:
                self.get_logger().warn('[MISSION] Goal rejected')
                callback(False)
                return
            self.get_logger().info('[MISSION] Goal accepted')
            handle.get_result_async().add_done_callback(
                lambda result: callback(navigation_succeeded(result.result())))
        future.add_done_callback(accepted)
        return True

    def _pickup_arrived(self, success):
        if not success:
            self.get_logger().error(f'{self.mission["id"]}: pickup navigation failed')
            self._transition(MissionState.ACCEPTED)
            return
        self._transition(MissionState.WAIT_FOR_LOADING)
        self.deadline = self.get_clock().now() + Duration(seconds=2.0)

    def _predock_arrived(self, success):
        if not success:
            self.get_logger().error(f'{self.mission["id"]}: pre-dock navigation failed')
            self._transition(MissionState.CARGO_LOADED)
            return
        if not self.dock_at_stations:
            self._transition(MissionState.UNLOADING)
            self.cargo.unload()
            self.deadline = self.get_clock().now() + Duration(seconds=2.0)
            return
        self._transition(MissionState.DOCKING)
        if not self.docking.dock(self.mission['destination'], self._docked):
            self.get_logger().warn('Waiting for dock_robot action server')
            self._transition(MissionState.CARGO_LOADED)

    def _docked(self, success):
        if success:
            # DockRobot finishes before the velocity smoother necessarily
            # reaches zero. Validate over a short settling window instead of
            # rejecting a physically successful dock on the first callback.
            self.dock_validation_deadline = (
                self.get_clock().now() + Duration(seconds=2.0))
            self.get_logger().info(
                '[MISSION] Dock action succeeded; validating stopped map-frame pose')
            return
        self.get_logger().error('[MISSION] Dock action reported failure')
        self._transition(MissionState.CARGO_LOADED)

    def _undocked(self, success):
        if success:
            self._transition(MissionState.COMPLETED)
            return
        self.get_logger().error('[MISSION] Undock action reported failure')
        self._transition(MissionState.UNLOADING)
        self.deadline = self.get_clock().now() + Duration(seconds=1.0)

    def _validate_docked_pose(self):
        target = self.stations[self.mission['destination']]
        pose = self._robot_pose_in_map()
        twist = self.odom.twist.twist if self.odom is not None else None
        valid = self.docking.validate(
            pose, twist, target, self.collision)
        if valid:
            position_error, yaw_error, speed, angular_speed = (
                self.docking.validation_errors(pose, twist, target))
            self.get_logger().info(
                '[MISSION] Dock validated in map frame: '
                f'position_error={position_error:.3f}m, '
                f'yaw_error={math.degrees(yaw_error):.2f}deg, '
                f'speed={speed:.3f}m/s, angular_speed={angular_speed:.3f}rad/s')
            self.dock_validation_deadline = None
            self._transition(MissionState.DOCKED)
            self.deadline = self.get_clock().now() + Duration(seconds=2.0)
            return
        if self.get_clock().now() < self.dock_validation_deadline:
            return
        details = 'pose unavailable'
        if pose is not None and twist is not None:
            position_error, yaw_error, speed, angular_speed = (
                self.docking.validation_errors(pose, twist, target))
            details = (
                f'position={position_error:.3f}m, '
                f'yaw={math.degrees(yaw_error):.2f}deg, '
                f'speed={speed:.3f}m/s, angular={angular_speed:.3f}rad/s, '
                f'collision={self.collision}')
        self.get_logger().error(
            '[MISSION] Dock validation failed after settling: ' + details)
        self.dock_validation_deadline = None
        self.get_logger().error(
            'Dock rejected: require position <0.08m, yaw <5deg, '
            'stopped, collision=false')
        self._transition(MissionState.CARGO_LOADED)

    def _robot_pose_in_map(self):
        """Transform the latest odometry pose into the stations' map frame."""
        if self.odom is None:
            self.get_logger().error('[MISSION] Dock validation has no odometry')
            return None
        source_frame = self.odom.header.frame_id or 'odom'
        pose = PoseStamped()
        pose.header = self.odom.header
        pose.header.frame_id = source_frame
        pose.pose = self.odom.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', source_frame, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose_stamped(pose, transform).pose
        except TransformException as error:
            self.get_logger().error(
                f'[MISSION] Cannot transform dock pose {source_frame}->map: {error}')
            return None

    def _tick(self):
        self._publish_state()
        if not self.started:
            return
        if self.state == MissionState.IDLE:
            self._transition(MissionState.ACCEPTED)
        elif self.state == MissionState.ACCEPTED:
            if not self._navigation_ready():
                return
            self._transition(MissionState.GO_TO_PICKUP)
            if not self._navigate(self.mission['pickup'], self._pickup_arrived, offset=-0.70):
                self._transition(MissionState.ACCEPTED)
        elif (self.state == MissionState.WAIT_FOR_LOADING and
              self.get_clock().now() >= self.deadline):
            self.cargo.load(self.mission['id'] + '_cargo')
            self._transition(MissionState.CARGO_LOADED)
        elif self.state == MissionState.CARGO_LOADED:
            self._transition(MissionState.GO_TO_PRE_DOCK)
            if not self._navigate(
                    self.mission['destination'], self._predock_arrived, offset=-0.70):
                self._transition(MissionState.CARGO_LOADED)
        elif (self.state == MissionState.DOCKING and
              self.dock_validation_deadline is not None):
            self._validate_docked_pose()
        elif self.state == MissionState.DOCKED and self.get_clock().now() >= self.deadline:
            self._transition(MissionState.UNLOADING)
            self.cargo.unload()
            self.deadline = self.get_clock().now() + Duration(seconds=1.0)
        elif self.state == MissionState.UNLOADING and self.get_clock().now() >= self.deadline:
            if not self.dock_at_stations:
                self._transition(MissionState.COMPLETED)
                return
            self._transition(MissionState.UNDOCKING)
            if not self.docking.undock(self._undocked):
                self.get_logger().warn('Waiting for undock_robot action server')
                self._transition(MissionState.UNLOADING)
                self.deadline = self.get_clock().now() + Duration(seconds=1.0)
        elif self.state == MissionState.COMPLETED:
            self._transition(MissionState.NEXT_MISSION)
        elif self.state == MissionState.NEXT_MISSION:
            self.mission_index += 1
            if self.mission_index >= len(self.missions):
                self.mission_index = len(self.missions) - 1
                self.get_logger().info('All warehouse missions M01-M04 completed')
                self.timer.cancel()
                return
            self._transition(MissionState.ACCEPTED)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()
