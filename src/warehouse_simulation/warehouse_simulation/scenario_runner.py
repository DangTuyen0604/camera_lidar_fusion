"""Region/state-triggered, repeatable warehouse scenario runner."""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import FusedDetection, FusedDetectionArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from simulation_interfaces.srv import DeleteEntity, SetEntityState, SpawnEntity
from std_msgs.msg import String, UInt64
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .pedestrian_roaming import RoamingAgent, WaypointGraph


class ScenarioRunner(Node):
    """Execute YAML actions through the simulator entity services."""

    def __init__(self):
        super().__init__('warehouse_scenario_runner')
        share = Path(get_package_share_directory('warehouse_simulation'))
        scenario_file = Path(self.declare_parameter(
            'scenario_file', str(share / 'config' / 'scenarios.yaml')).value)
        stations_file = Path(self.declare_parameter(
            'stations_file', str(share / 'config' / 'stations.yaml')).value)
        scenario_name = self.declare_parameter('scenario', 'warehouse_demo').value
        scenario_document = yaml.safe_load(scenario_file.read_text())
        self.actions = scenario_document['scenarios'][scenario_name]
        self.actor_names = self._validate_actors(self.actions)
        self.waypoint_graphs = {
            name: WaypointGraph(document)
            for name, document in scenario_document.get(
                'pedestrian_waypoint_graphs', {}).items()
        }
        self.spawned_actors = set()
        self.scenario_failed = False
        self.scenario_seed = int(self.declare_parameter(
            'random_seed', int(scenario_document['seed'])).value)
        self.action_timeout_sec = float(scenario_document['action_timeout_sec'])
        self.regions = yaml.safe_load(stations_file.read_text()).get('regions', {})
        self.models_dir = share / 'models'
        self.index = 0
        self.pending = None
        self.robot_xy = None
        self.robot_yaw = 0.0
        # Simulator ground truth of the robot, when warehouse_mujoco provides
        # it. Scenario physics (worker avoidance, spawns, carried cargo) must
        # follow the real robot, not AMCL: a few cm of AMCL error let a
        # kinematic worker walk into the robot, the shove broke wheel
        # odometry, and AMCL then diverged by metres.
        self.has_ground_truth = False
        self.map_offset = (
            float(self.declare_parameter('map_offset_x', 0.0).value),
            float(self.declare_parameter('map_offset_y', -4.0).value))
        self.mission_state = ''
        self.attached = set()
        self.paths = {}
        self.roamers = {}
        self.enable_roaming = bool(self.declare_parameter(
            'enable_roaming', True).value)
        self.worker_clearance = float(self.declare_parameter(
            'worker_clearance', 0.80).value)
        self.robot_clearance = float(self.declare_parameter(
            'robot_clearance', 0.65).value)
        # Kinematic entities cannot be pushed; never spawn one onto the robot.
        self.spawn_clearance = float(self.declare_parameter(
            'spawn_clearance', 1.0).value)
        self.update_period = float(self.declare_parameter(
            'update_period_sec', 0.20).value)
        if not 0.05 <= self.update_period <= 0.50:
            raise ValueError(
                'update_period_sec must be between 0.05 and 0.50 seconds')
        self.motion_futures = {}
        self.entities = {}
        self.collision_count = 0
        self.in_collision = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publish_fused_detections = bool(self.declare_parameter(
            'publish_fused_detections', True).value)
        # simulation_interfaces entity services served by warehouse_mujoco.
        services = self.declare_parameter(
            'simulator_services', '/mujoco').value.rstrip('/')
        self.spawn_client = self.create_client(
            SpawnEntity, f'{services}/spawn_entity')
        self.pose_client = self.create_client(
            SetEntityState, f'{services}/set_entity_state')
        self.delete_client = self.create_client(
            DeleteEntity, f'{services}/delete_entity')
        self.detection_publisher = None
        if self.publish_fused_detections:
            self.detection_publisher = self.create_publisher(
                FusedDetectionArray, '/fusion/detections_3d', 10)
        self.truth_publisher = self.create_publisher(
            FusedDetectionArray, '/benchmark/ground_truth/detections_3d', 10)
        self.event_publisher = self.create_publisher(String, '/benchmark/events', 10)
        self.collision_publisher = self.create_publisher(
            UInt64, '/benchmark/collision_count', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(
            Odometry, '/ground_truth/odom', self._ground_truth, 10)
        self.create_subscription(String, '/mission/state', self._state, 10)
        self.create_timer(self.update_period, self._tick)
        self.create_timer(self.update_period, self._publish_detections)

    @staticmethod
    def _validate_actors(actions):
        """Validate every configured worker without imposing a fixed count."""
        actors = [
            action for action in actions
            if action.get('action') == 'spawn'
            and action.get('model') == 'worker'
        ]
        names = [action.get('name') for action in actors]
        if any(not name for name in names):
            raise ValueError('Every worker spawn must have a non-empty name')
        if len(names) != len(set(names)):
            raise ValueError(f'Worker entity names must be unique: {names}')

        poses = []
        for action in actors:
            pose = action.get('pose')
            if not isinstance(pose, list) or len(pose) != 4:
                raise ValueError(
                    f"Worker {action['name']} must have pose [x, y, z, yaw]")
            poses.append(tuple(float(value) for value in pose[:3]))
        if len(poses) != len(set(poses)):
            raise ValueError(f'Workers must have different initial poses: {poses}')

        motion_names = [
            action.get('name') for action in actions
            if action.get('action') in ('follow_path', 'roam')
        ]
        missing_motion = sorted(set(names) - set(motion_names))
        duplicate_motion = sorted({
            name for name in motion_names if motion_names.count(name) > 1})
        if missing_motion:
            raise ValueError(
                f'Workers missing independent motion actions: {missing_motion}')
        if duplicate_motion:
            raise ValueError(
                f'Workers have multiple motion actions: {duplicate_motion}')
        return frozenset(names)

    def _ground_truth(self, msg):
        # The static map is rasterized from the same world, so world == map.
        self.has_ground_truth = True
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom(self, msg):
        if self.has_ground_truth:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = msg.header.frame_id or 'odom'
        pose.pose = msg.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', pose.header.frame_id, Time())
            map_pose = do_transform_pose_stamped(pose, transform).pose
            self.robot_xy = (map_pose.position.x, map_pose.position.y)
            q = map_pose.orientation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except TransformException:
            # Standalone warehouse.launch has no localization stack. Keep its
            # explicit spawn-origin fallback, but always prefer TF when map is
            # available in the full demo.
            self.robot_xy = (
                msg.pose.pose.position.x + self.map_offset[0],
                msg.pose.pose.position.y + self.map_offset[1])
            q = msg.pose.pose.orientation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _state(self, msg):
        self.mission_state = msg.data

    @staticmethod
    def _pose(values):
        x, y, z, yaw = values
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _start(self, client, request, description, advance=True, context=None):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        if advance:
            self.pending = (
                future, description, time.monotonic(), context or {})
        return future

    def _fail_command(self, description, reason, context):
        self.scenario_failed = True
        if context.get('operation') == 'spawn':
            self.get_logger().error(
                f"[SCENARIO] Spawn failed: {context['name']}\nReason: {reason}")
        else:
            self.get_logger().error(
                f'[SCENARIO] Simulator command failed: {description}\nReason: {reason}')

    def _move(self, name, pose, advance=True):
        motion = self.motion_futures.get(name)
        # Never replace an unconsumed response, even if it is already done.
        if motion is not None:
            return False
        request = SetEntityState.Request()
        request.entity = name
        request.state.pose = self._pose(pose)
        context = {
            'operation': 'move',
            'name': name,
            'pose': list(pose),
        }
        future = self._start(
            self.pose_client, request, f'move {name}', advance, context)
        if future is None:
            return False
        if not advance:
            self.motion_futures[name] = (
                future, time.monotonic(), list(pose))
        return True

    def _finish_motion(self, name):
        """Consume a completed SetEntityState request without hiding failures."""
        motion = self.motion_futures.get(name)
        if motion is None:
            return True
        future, started, pose = motion
        if not future.done():
            if time.monotonic() - started > self.action_timeout_sec:
                future.cancel()
                del self.motion_futures[name]
                self._fail_command(
                    f'move {name}',
                    f'timed out after {self.action_timeout_sec}s',
                    {'operation': 'move', 'name': name})
            return False
        del self.motion_futures[name]
        try:
            response = future.result()
        except Exception as error:
            self._fail_command(
                f'move {name}', repr(error),
                {'operation': 'move', 'name': name})
            return False
        if response is None or response.result.result != response.result.RESULT_OK:
            reason = ('service returned no response' if response is None else
                      response.result.error_message or
                      f'result code {response.result.result}')
            self._fail_command(
                f'move {name}', reason,
                {'operation': 'move', 'name': name})
            return False
        if name in self.entities:
            self.entities[name]['pose'] = pose
        return True

    def _publish_detections(self):
        """
        Publish deterministic simulator truth through the real fusion contract.

        This keeps the navigation/costmap integration deterministic while the
        simulated camera and lidar continue to publish their physical sensor data.
        """
        message = FusedDetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        class_ids = {'person': 0, 'worker': 0, 'box': 1, 'cargo_box': 1, 'pallet': 2}
        for name, entity in self.entities.items():
            if name in self.attached:
                # Carried cargo moves with the robot; it is not an obstacle.
                continue
            detection = FusedDetection()
            detection.header = message.header
            detection.detection.class_name = entity['class_name']
            detection.detection.class_id = class_ids.get(entity['class_name'], 99)
            detection.detection.confidence = 0.99
            detection.position.x = float(entity['pose'][0])
            detection.position.y = float(entity['pose'][1])
            detection.position.z = float(entity['pose'][2])
            detection.depth = math.hypot(detection.position.x, detection.position.y)
            detection.lidar_point_count = 32
            detection.valid = True
            message.detections.append(detection)
        if self.detection_publisher is not None:
            self.detection_publisher.publish(message)
        self.truth_publisher.publish(message)
        collision = UInt64()
        collision.data = self.collision_count
        self.collision_publisher.publish(collision)

    def _tick_paths_and_cargo(self):
        for name, path in list(self.paths.items()):
            if not self._finish_motion(name):
                continue
            x, y, speed, targets, loop_targets = path
            tx, ty = targets[0]
            distance = math.hypot(tx - x, ty - y)
            if distance < 0.08:
                targets.pop(0)
                if not targets:
                    if loop_targets:
                        targets.extend([list(point) for point in loop_targets])
                    else:
                        del self.paths[name]
                        continue
                tx, ty = targets[0]
                distance = math.hypot(tx - x, ty - y)
            yaw = math.atan2(ty - y, tx - x)
            step = min(speed * self.update_period, distance)
            x += step * (tx - x) / max(distance, 1e-6)
            y += step * (ty - y) / max(distance, 1e-6)
            self.paths[name] = [x, y, speed, targets, loop_targets]
            self._move(name, [x, y, 0.0, yaw], advance=False)
        self._tick_roamers()
        if self.robot_xy:
            for name in self.attached:
                if self._finish_motion(name):
                    offset = -0.25
                    # Carry the 0.50 m box on the deck above the ~0.41 m LiDAR
                    # plane (bottom at 0.60 m). Lower, the scan hits the robot's
                    # own cargo: phantom obstacles inside the footprint and a
                    # corrupted AMCL match that drags the box further off.
                    self._move(
                        name,
                        [self.robot_xy[0] + offset * math.cos(self.robot_yaw),
                         self.robot_xy[1] + offset * math.sin(self.robot_yaw),
                         0.85, self.robot_yaw],
                        advance=False)
            colliding = any(
                math.hypot(self.robot_xy[0] - entity['pose'][0],
                           self.robot_xy[1] - entity['pose'][1]) < 0.42
                for name, entity in self.entities.items() if name not in self.attached)
            if colliding and not self.in_collision:
                self.collision_count += 1
            self.in_collision = colliding

    def _roaming_pose_is_clear(self, name, x, y):
        if self.robot_xy and math.hypot(
                x - self.robot_xy[0], y - self.robot_xy[1]) < self.robot_clearance:
            return False
        for other_name, entity in self.entities.items():
            if (other_name != name and entity['class_name'] == 'worker' and
                    math.hypot(x - entity['pose'][0],
                               y - entity['pose'][1]) < self.worker_clearance):
                return False
        return True

    def _tick_roamers(self):
        """Advance every pedestrian independently of scenario action waits."""
        now = time.monotonic()
        for name, agent in list(self.roamers.items()):
            if not self._finish_motion(name) or now < agent.pause_until:
                continue
            entity = self.entities.get(name)
            if entity is None:
                continue
            x, y = entity['pose'][:2]
            if not agent.route:
                destination = agent.choose_trip()
                self.get_logger().info(
                    f'[SCENARIO] Roam destination: {name} -> {destination} '
                    f'({agent.speed:.2f} m/s)')
            target_name = agent.route[0]
            tx, ty = agent.graph.nodes[target_name]
            distance = math.hypot(tx - x, ty - y)
            if distance < 0.08:
                x, y = tx, ty
                agent.current_node = target_name
                agent.route.pop(0)
                if not agent.route:
                    agent.arrive(now)
                    self.get_logger().info(
                        f'[SCENARIO] Roam arrived: {name} at {target_name}; '
                        f'pause until {agent.pause_until:.2f}')
                    continue
                target_name = agent.route[0]
                tx, ty = agent.graph.nodes[target_name]
                distance = math.hypot(tx - x, ty - y)
            yaw = math.atan2(ty - y, tx - x)
            step = min(agent.speed * self.update_period, distance)
            next_x = x + step * (tx - x) / max(distance, 1e-6)
            next_y = y + step * (ty - y) / max(distance, 1e-6)
            if not self._roaming_pose_is_clear(name, next_x, next_y):
                if agent.wait_or_retreat(now):
                    self.get_logger().info(
                        f'[SCENARIO] Roam yielding: {name} retreats toward '
                        f'{agent.current_node}')
                continue
            agent.blocked_since = None
            self._move(name, [next_x, next_y, 0.0, yaw], advance=False)

    def _tick(self):
        if self.scenario_failed:
            return
        self._tick_paths_and_cargo()
        if self.scenario_failed:
            return
        if self.pending is not None:
            future, description, started, context = self.pending
            if not future.done():
                if time.monotonic() - started > self.action_timeout_sec:
                    future.cancel()
                    self.pending = None
                    self._fail_command(
                        description,
                        f'timed out after {self.action_timeout_sec}s; '
                        f'scenario paused (seed={self.scenario_seed})',
                        context)
                return
            self.pending = None
            try:
                response = future.result()
            except Exception as error:  # The exception is logged and pauses the scenario.
                self._fail_command(description, repr(error), context)
                return
            if response is not None and response.result.result == response.result.RESULT_OK:
                if context.get('operation') == 'spawn':
                    name = context['name']
                    actual_name = response.entity_name or name
                    if actual_name != name:
                        self._fail_command(
                            description,
                            f'Simulator returned unexpected entity name {actual_name!r}',
                            context)
                        return
                    model = context['model']
                    if model in ('pallet', 'cargo_box', 'worker'):
                        self.entities[name] = {
                            'class_name': model,
                            'pose': list(context['pose']),
                        }
                    self.get_logger().info(f'[SCENARIO] Spawn success: {name}')
                    if name in self.actor_names:
                        self.spawned_actors.add(name)
                        if len(self.spawned_actors) == len(self.actor_names):
                            self.get_logger().info(
                                '[SCENARIO] Actors ready: '
                                f'{len(self.spawned_actors)}/{len(self.actor_names)}')
                elif context.get('operation') == 'move':
                    name = context['name']
                    if name in self.entities:
                        self.entities[name]['pose'] = list(context['pose'])
                    self.get_logger().info(f'[SCENARIO] {description}')
                else:
                    self.get_logger().info(f'[SCENARIO] {description}')
                event = String()
                event.data = description.replace(' ', ':', 1)
                self.event_publisher.publish(event)
                self.index += 1
            else:
                if response is None:
                    reason = 'service returned no response'
                else:
                    reason = response.result.error_message or (
                        f'result code {response.result.result}')
                self._fail_command(description, reason, context)
            return
        if self.index >= len(self.actions):
            return
        action = self.actions[self.index]
        kind = action['action']
        if kind == 'wait_for_robot_region':
            if self.robot_xy:
                xmin, xmax, ymin, ymax = self.regions[action['region']]
                x, y = self.robot_xy
                if xmin <= x <= xmax and ymin <= y <= ymax:
                    self.get_logger().info(f"entered region {action['region']}")
                    self.index += 1
        elif kind == 'wait_for_mission_state':
            if self.mission_state == action['state']:
                self.index += 1
        elif kind == 'spawn':
            model = action['model']
            if (model != 'worker' and self.robot_xy and math.hypot(
                    action['pose'][0] - self.robot_xy[0],
                    action['pose'][1] - self.robot_xy[1]) < self.spawn_clearance):
                return
            request = SpawnEntity.Request()
            request.name = action['name']
            request.allow_renaming = False
            request.resource_string = (
                self.models_dir / model / 'model.sdf').read_text()
            request.initial_pose.header.frame_id = 'world'
            request.initial_pose.pose = self._pose(action['pose'])
            if self.spawn_client.service_is_ready():
                name = action['name']
                if name in self.actor_names:
                    self.get_logger().info(
                        f'[SCENARIO] Spawning actor: {name}')
                else:
                    self.get_logger().info(
                        f'[SCENARIO] Spawning entity: {name}')
                self._start(
                    self.spawn_client, request, f'spawn {name}',
                    context={
                        'operation': 'spawn',
                        'name': name,
                        'model': model,
                        'pose': list(action['pose']),
                    })
        elif kind == 'delete':
            request = DeleteEntity.Request()
            request.entity = action['name']
            if self._start(self.delete_client, request, f"delete {action['name']}"):
                self.paths.pop(action['name'], None)
                self.motion_futures.pop(action['name'], None)
                self.entities.pop(action['name'], None)
        elif kind == 'move':
            self._move(action['name'], action['pose'])
        elif kind == 'follow_path':
            points = [list(point) for point in action['path']]
            first = points[0]
            targets = points[1:]
            loop_targets = ([list(point) for point in targets]
                            if action.get('loop', False) else [])
            self.paths[action['name']] = [
                first[0], first[1], action['speed'], targets, loop_targets]
            self.index += 1
        elif kind == 'roam':
            name = action['name']
            if self.enable_roaming:
                graph_name = action['graph']
                graph = self.waypoint_graphs[graph_name]
                initial_xy = self.entities[name]['pose'][:2]
                self.roamers[name] = RoamingAgent(
                    name, graph, self.scenario_seed, initial_xy,
                    action.get('speed_range', [0.4, 0.8]),
                    action.get('pause_range', [0.5, 2.0]))
                self.get_logger().info(
                    f'[SCENARIO] Roaming enabled: {name} on {graph_name}')
            else:
                self.get_logger().info(
                    f'[SCENARIO] Roaming disabled: {name}')
            self.index += 1
        elif kind == 'attach_cargo':
            self.attached.add(action['name'])
            self.index += 1
        elif kind == 'detach_cargo':
            self.attached.discard(action['name'])
            self.index += 1
        else:
            self.get_logger().error(f'Unknown action {kind}; scenario paused')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
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
