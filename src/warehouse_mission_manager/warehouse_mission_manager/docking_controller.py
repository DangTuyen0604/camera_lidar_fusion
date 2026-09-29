import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, UndockRobot
from rclpy.action import ActionClient


class DockingController:
    POSITION_TOLERANCE = 0.08
    YAW_TOLERANCE = math.radians(5.0)
    VELOCITY_TOLERANCE = 0.02

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, DockRobot, 'dock_robot')
        self.undock_client = ActionClient(node, UndockRobot, 'undock_robot')

    def dock(self, dock_id, result_callback):
        if not self.client.wait_for_server(timeout_sec=1.0):
            return False
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = False
        future = self.client.send_goal_async(goal)

        def accepted(done):
            try:
                handle = done.result()
            except Exception as error:
                self.node.get_logger().error(
                    f'[MISSION] Dock goal request failed: {error!r}')
                result_callback(False)
                return
            if not handle.accepted:
                self.node.get_logger().warn(
                    f'[MISSION] Dock goal rejected: {dock_id}')
                result_callback(False)
                return
            self.node.get_logger().info(
                f'[MISSION] Dock goal accepted: {dock_id}')

            def completed(done_result):
                try:
                    response = done_result.result()
                    result = response.result
                    if result.success:
                        self.node.get_logger().info(
                            f'[MISSION] Dock action completed: {dock_id}; '
                            f'retries={result.num_retries}')
                    else:
                        self.node.get_logger().error(
                            f'[MISSION] Dock action failed: {dock_id}; '
                            f'error_code={result.error_code}, '
                            f'retries={result.num_retries}, '
                            f'error={result.error_msg!r}')
                    result_callback(bool(result.success))
                except Exception as error:
                    self.node.get_logger().error(
                        f'[MISSION] Dock result failed: {error!r}')
                    result_callback(False)

            handle.get_result_async().add_done_callback(completed)
        future.add_done_callback(accepted)
        return True

    def undock(self, result_callback):
        """Leave dock contact and return to the plugin's staging pose."""
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            return False
        goal = UndockRobot.Goal()
        goal.dock_type = 'warehouse_dock'
        goal.max_undocking_time = 60.0
        future = self.undock_client.send_goal_async(goal)

        def accepted(done):
            try:
                handle = done.result()
            except Exception as error:
                self.node.get_logger().error(
                    f'[MISSION] Undock goal request failed: {error!r}')
                result_callback(False)
                return
            if not handle.accepted:
                self.node.get_logger().warn('[MISSION] Undock goal rejected')
                result_callback(False)
                return
            self.node.get_logger().info('[MISSION] Undock goal accepted')

            def completed(done_result):
                try:
                    result = done_result.result().result
                    if result.success:
                        self.node.get_logger().info(
                            '[MISSION] Undock action completed at staging pose')
                    else:
                        self.node.get_logger().error(
                            '[MISSION] Undock action failed: '
                            f'error_code={result.error_code}, '
                            f'error={result.error_msg!r}')
                    result_callback(bool(result.success))
                except Exception as error:
                    self.node.get_logger().error(
                        f'[MISSION] Undock result failed: {error!r}')
                    result_callback(False)

            handle.get_result_async().add_done_callback(completed)

        future.add_done_callback(accepted)
        return True

    @staticmethod
    def validation_errors(pose, twist, target):
        """Return pose and velocity errors for an already common pose frame."""
        dx, dy = pose.position.x - target['x'], pose.position.y - target['y']
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_error = abs(math.atan2(
            math.sin(yaw - target['yaw']),
            math.cos(yaw - target['yaw'])))
        speed = math.hypot(twist.linear.x, twist.linear.y)
        return math.hypot(dx, dy), yaw_error, speed, abs(twist.angular.z)

    @staticmethod
    def validate(pose, twist, target, collision):
        """Validate a stopped dock pose; pose must be in the map frame."""
        if pose is None or twist is None or collision:
            return False
        position_error, yaw_error, speed, angular_speed = (
            DockingController.validation_errors(pose, twist, target))
        return (position_error < DockingController.POSITION_TOLERANCE and
                yaw_error < DockingController.YAW_TOLERANCE and
                speed < DockingController.VELOCITY_TOLERANCE and
                angular_speed < DockingController.VELOCITY_TOLERANCE)


def pose_stamped(node, station, offset=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = station['x'] + offset * math.cos(station['yaw'])
    pose.pose.position.y = station['y'] + offset * math.sin(station['yaw'])
    pose.pose.orientation.z = math.sin(station['yaw'] / 2.0)
    pose.pose.orientation.w = math.cos(station['yaw'] / 2.0)
    return pose
