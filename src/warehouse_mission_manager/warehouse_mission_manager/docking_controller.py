import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
from rclpy.action import ActionClient


class DockingController:
    POSITION_TOLERANCE = 0.08
    YAW_TOLERANCE = math.radians(5.0)
    VELOCITY_TOLERANCE = 0.02

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, DockRobot, 'dock_robot')

    def dock(self, dock_id, result_callback):
        if not self.client.wait_for_server(timeout_sec=1.0):
            return False
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = dock_id
        goal.navigate_to_staging_pose = False
        future = self.client.send_goal_async(goal)

        def accepted(done):
            handle = done.result()
            if not handle.accepted:
                result_callback(False)
                return
            handle.get_result_async().add_done_callback(
                lambda result: result_callback(bool(result.result().result.success)))
        future.add_done_callback(accepted)
        return True

    @staticmethod
    def validate(odom, target, collision):
        if odom is None or collision:
            return False
        pose = odom.pose.pose
        dx, dy = pose.position.x - target['x'], pose.position.y - target['y']
        q = pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        yaw_error = abs(math.atan2(math.sin(yaw - target['yaw']), math.cos(yaw - target['yaw'])))
        twist = odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        return (math.hypot(dx, dy) < DockingController.POSITION_TOLERANCE and
                yaw_error < DockingController.YAW_TOLERANCE and
                speed < DockingController.VELOCITY_TOLERANCE and
                abs(twist.angular.z) < DockingController.VELOCITY_TOLERANCE)


def pose_stamped(node, station, offset=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = station['x'] + offset * math.cos(station['yaw'])
    pose.pose.position.y = station['y'] + offset * math.sin(station['yaw'])
    pose.pose.orientation.z = math.sin(station['yaw'] / 2.0)
    pose.pose.orientation.w = math.cos(station['yaw'] / 2.0)
    return pose
