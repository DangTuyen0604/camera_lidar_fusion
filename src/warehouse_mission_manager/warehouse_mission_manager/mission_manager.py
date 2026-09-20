"""M01-M04 action-driven warehouse mission state machine."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .cargo_manager import CargoManager
from .docking_controller import DockingController, pose_stamped
from .mission_state import MissionState


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
        self.mission_index = 0
        self.odom = None
        self.collision = False
        self.deadline = None
        self.cargo = CargoManager()
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose')
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

    def _navigate(self, station_name, callback, offset=0.0):
        if not self.navigation.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for navigate_to_pose action server')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped(self, self.stations[station_name], offset)
        future = self.navigation.send_goal_async(goal)

        def accepted(done):
            handle = done.result()
            if not handle.accepted:
                callback(False)
                return
            handle.get_result_async().add_done_callback(
                lambda result: callback(result.result().result.error_code == 0))
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
        self._transition(MissionState.DOCKING)
        if not self.docking.dock(self.mission['destination'], self._docked):
            self.get_logger().warn('Waiting for dock_robot action server')
            self._transition(MissionState.CARGO_LOADED)

    def _docked(self, success):
        target = self.stations[self.mission['destination']]
        valid = success and self.docking.validate(self.odom, target, self.collision)
        if not valid:
            self.get_logger().error(
                'Dock rejected: require position <0.08m, yaw <5deg, stopped, collision=false')
            self._transition(MissionState.CARGO_LOADED)
            return
        self._transition(MissionState.DOCKED)
        self.deadline = self.get_clock().now() + Duration(seconds=2.0)

    def _tick(self):
        self._publish_state()
        if not self.started:
            return
        if self.state == MissionState.IDLE:
            self._transition(MissionState.ACCEPTED)
        elif self.state == MissionState.ACCEPTED:
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
        elif self.state == MissionState.DOCKED and self.get_clock().now() >= self.deadline:
            self._transition(MissionState.UNLOADING)
            self.cargo.unload()
            self.deadline = self.get_clock().now() + Duration(seconds=1.0)
        elif self.state == MissionState.UNLOADING and self.get_clock().now() >= self.deadline:
            self._transition(MissionState.COMPLETED)
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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
