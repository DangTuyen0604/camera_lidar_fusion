"""Region/state-triggered, repeatable Gazebo warehouse scenario runner."""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from fusion_interfaces.msg import FusedDetection, FusedDetectionArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import DeleteEntity, SetEntityState, SpawnEntity
from std_msgs.msg import String, UInt64
import yaml


class ScenarioRunner(Node):
    """Execute YAML actions through Gazebo transport helper executables."""

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
        self.scenario_seed = int(scenario_document['seed'])
        self.action_timeout_sec = float(scenario_document['action_timeout_sec'])
        self.regions = yaml.safe_load(stations_file.read_text()).get('regions', {})
        self.models_dir = share / 'models'
        self.index = 0
        self.pending = None
        self.robot_xy = None
        self.map_offset = (
            float(self.declare_parameter('map_offset_x', 0.0).value),
            float(self.declare_parameter('map_offset_y', -4.0).value))
        self.mission_state = ''
        self.attached = set()
        self.paths = {}
        self.motion_futures = {}
        self.entities = {}
        self.collision_count = 0
        self.in_collision = False
        self.spawn_client = self.create_client(SpawnEntity, '/gzserver/spawn_entity')
        self.pose_client = self.create_client(SetEntityState, '/gzserver/set_entity_state')
        self.delete_client = self.create_client(DeleteEntity, '/gzserver/delete_entity')
        self.detection_publisher = self.create_publisher(
            FusedDetectionArray, '/fusion/detections_3d', 10)
        self.truth_publisher = self.create_publisher(
            FusedDetectionArray, '/benchmark/ground_truth/detections_3d', 10)
        self.event_publisher = self.create_publisher(String, '/benchmark/events', 10)
        self.collision_publisher = self.create_publisher(
            UInt64, '/benchmark/collision_count', 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(String, '/mission/state', self._state, 10)
        self.create_timer(0.10, self._tick)
        self.create_timer(0.10, self._publish_detections)

    def _odom(self, msg):
        self.robot_xy = (
            msg.pose.pose.position.x + self.map_offset[0],
            msg.pose.pose.position.y + self.map_offset[1])

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

    def _start(self, client, request, description, advance=True):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        if advance:
            self.pending = (future, description, time.monotonic())
        return future

    def _move(self, name, pose, advance=True):
        future = self.motion_futures.get(name)
        if future is not None and not future.done():
            return False
        request = SetEntityState.Request()
        request.entity = name
        request.state.pose = self._pose(pose)
        future = self._start(self.pose_client, request, f'move {name}', advance)
        if future is None:
            return False
        if not advance:
            self.motion_futures[name] = future
        if name in self.entities:
            self.entities[name]['pose'] = list(pose)
        return True

    def _publish_detections(self):
        """
        Publish deterministic simulator truth through the real fusion contract.

        This keeps the navigation/costmap integration deterministic while the
        Gazebo camera and lidar continue to publish their physical sensor data.
        """
        message = FusedDetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        class_ids = {'person': 0, 'worker': 0, 'box': 1, 'cargo_box': 1, 'pallet': 2}
        for entity in self.entities.values():
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
        self.detection_publisher.publish(message)
        self.truth_publisher.publish(message)
        collision = UInt64()
        collision.data = self.collision_count
        self.collision_publisher.publish(collision)

    def _tick_paths_and_cargo(self):
        for name, path in list(self.paths.items()):
            future = self.motion_futures.get(name)
            if future is not None and not future.done():
                continue
            x, y, speed, targets = path
            tx, ty = targets[0]
            distance = math.hypot(tx - x, ty - y)
            if distance < 0.08:
                targets.pop(0)
                if not targets:
                    del self.paths[name]
                    continue
                tx, ty = targets[0]
                distance = math.hypot(tx - x, ty - y)
            step = min(speed * 0.1, distance)
            x += step * (tx - x) / max(distance, 1e-6)
            y += step * (ty - y) / max(distance, 1e-6)
            self.paths[name] = [x, y, speed, targets]
            self._move(name, [x, y, 0.0, 0.0], advance=False)
        if self.robot_xy:
            for name in self.attached:
                self._move(
                    name, [self.robot_xy[0] - 0.25, self.robot_xy[1], 0.48, 0.0],
                    advance=False)
            colliding = any(
                math.hypot(self.robot_xy[0] - entity['pose'][0],
                           self.robot_xy[1] - entity['pose'][1]) < 0.42
                for name, entity in self.entities.items() if name not in self.attached)
            if colliding and not self.in_collision:
                self.collision_count += 1
            self.in_collision = colliding

    def _tick(self):
        self._tick_paths_and_cargo()
        if self.pending is not None:
            future, description, started = self.pending
            if not future.done():
                if time.monotonic() - started > self.action_timeout_sec:
                    future.cancel()
                    self.pending = None
                    self.get_logger().error(
                        f'Gazebo command timed out after {self.action_timeout_sec}s: '
                        f'{description}; scenario paused (seed={self.scenario_seed})')
                return
            self.pending = None
            response = future.result()
            if response is not None and response.result.result == response.result.RESULT_OK:
                self.get_logger().info(description)
                event = String()
                event.data = description.replace(' ', ':', 1)
                self.event_publisher.publish(event)
                self.index += 1
            else:
                self.get_logger().error(f'Gazebo command failed: {description}')
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
            request = SpawnEntity.Request()
            request.name = action['name']
            request.resource_string = (
                self.models_dir / model / 'model.sdf').read_text()
            request.initial_pose.header.frame_id = 'world'
            request.initial_pose.pose = self._pose(action['pose'])
            if self._start(self.spawn_client, request, f"spawn {action['name']}"):
                if model in ('pallet', 'cargo_box', 'worker'):
                    self.entities[action['name']] = {
                        'class_name': model, 'pose': list(action['pose'])}
        elif kind == 'delete':
            request = DeleteEntity.Request()
            request.entity = action['name']
            if self._start(self.delete_client, request, f"delete {action['name']}"):
                self.entities.pop(action['name'], None)
        elif kind == 'move':
            self._move(action['name'], action['pose'])
        elif kind == 'follow_path':
            first = action['path'][0]
            self.paths[action['name']] = [
                first[0], first[1], action['speed'], list(action['path'][1:])]
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
