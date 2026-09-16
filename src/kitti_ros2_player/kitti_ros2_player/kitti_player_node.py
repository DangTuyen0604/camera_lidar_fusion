from pathlib import Path

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from kitti_ros2_player.calibration_parser import (
    load_camera_info,
    load_camera_info_yaml,
    load_lidar_camera_calibration,
    load_lidar_camera_calibration_yaml,
    matrix_to_quaternion,
)
from kitti_ros2_player.image_loader import ImageLoader, TimestampSequence
from kitti_ros2_player.pointcloud_loader import (
    create_pointcloud2,
    draw_lidar_overlay,
    PointCloudLoader,
    project_velodyne_to_image,
)
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class KittiPlayerNode(Node):

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'kitti_player_node',
            parameter_overrides=parameter_overrides,
        )

        default_image_dir = (
            Path.cwd()
            / 'data'
            / 'kitti'
            / '2011_09_26'
            / '2011_09_26_drive_0005_sync'
            / 'image_02'
            / 'data'
        )
        default_calibration_path = (
            Path.cwd()
            / 'data'
            / 'kitti'
            / '2011_09_26'
            / 'calib_cam_to_cam.txt'
        )
        default_pointcloud_dir = (
            Path.cwd()
            / 'data'
            / 'kitti'
            / '2011_09_26'
            / '2011_09_26_drive_0005_sync'
            / 'velodyne_points'
            / 'data'
        )
        default_velodyne_calibration_path = (
            Path.cwd()
            / 'data'
            / 'kitti'
            / '2011_09_26'
            / 'calib_velo_to_cam.txt'
        )
        default_image_timestamps_path = (
            default_image_dir.parent / 'timestamps.txt'
        )
        default_pointcloud_timestamps_path = (
            default_pointcloud_dir.parent / 'timestamps.txt'
        )
        default_config_dir = (
            Path.cwd() / 'src' / 'fusion_bringup' / 'config'
        )
        default_camera_intrinsics_yaml = (
            default_config_dir / 'camera_intrinsics.yaml'
        )
        default_extrinsics_yaml = (
            default_config_dir / 'lidar_camera_extrinsics.yaml'
        )

        self.declare_parameter('image_dir', str(default_image_dir))
        self.declare_parameter(
            'calibration_path',
            str(default_calibration_path),
        )
        self.declare_parameter(
            'pointcloud_dir',
            str(default_pointcloud_dir),
        )
        self.declare_parameter(
            'velodyne_calibration_path',
            str(default_velodyne_calibration_path),
        )
        self.declare_parameter(
            'image_timestamps_path',
            str(default_image_timestamps_path),
        )
        self.declare_parameter(
            'pointcloud_timestamps_path',
            str(default_pointcloud_timestamps_path),
        )
        self.declare_parameter('calibration_format', 'kitti')
        self.declare_parameter(
            'camera_intrinsics_yaml',
            str(default_camera_intrinsics_yaml),
        )
        self.declare_parameter(
            'extrinsics_yaml',
            str(default_extrinsics_yaml),
        )
        self.declare_parameter('loop', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('startup_delay_sec', 0.0)
        self.declare_parameter('timestamp_policy', 'rebase_kitti')
        self.declare_parameter('max_sensor_time_offset_sec', 0.05)
        self.declare_parameter('max_projection_depth', 80.0)
        self.declare_parameter('projection_point_radius', 2)
        self.declare_parameter('camera_frame_id', 'camera_optical_frame')
        self.declare_parameter('lidar_frame_id', 'velodyne')

        image_dir = self.get_parameter('image_dir').value
        calibration_path = self.get_parameter('calibration_path').value
        pointcloud_dir = self.get_parameter('pointcloud_dir').value
        velodyne_calibration_path = self.get_parameter(
            'velodyne_calibration_path'
        ).value
        image_timestamps_path = self.get_parameter(
            'image_timestamps_path'
        ).value
        pointcloud_timestamps_path = self.get_parameter(
            'pointcloud_timestamps_path'
        ).value
        self.calibration_format = self.get_parameter(
            'calibration_format'
        ).value
        camera_intrinsics_yaml = self.get_parameter(
            'camera_intrinsics_yaml'
        ).value
        extrinsics_yaml = self.get_parameter('extrinsics_yaml').value
        loop = self.get_parameter('loop').value
        publish_rate = self.get_parameter('publish_rate').value
        startup_delay_sec = self.get_parameter('startup_delay_sec').value
        self.timestamp_policy = self.get_parameter(
            'timestamp_policy'
        ).value
        max_sensor_time_offset_sec = self.get_parameter(
            'max_sensor_time_offset_sec'
        ).value
        self.max_projection_depth = self.get_parameter(
            'max_projection_depth'
        ).value
        self.projection_point_radius = self.get_parameter(
            'projection_point_radius'
        ).value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value

        if not np.isfinite(publish_rate) or publish_rate <= 0.0:
            raise RuntimeError(
                f'publish_rate must be greater than zero: {publish_rate}'
            )
        if not np.isfinite(startup_delay_sec) or startup_delay_sec < 0.0:
            raise RuntimeError(
                'startup_delay_sec must be finite and non-negative: '
                f'{startup_delay_sec}'
            )

        if self.max_projection_depth <= 0.1:
            raise RuntimeError(
                'max_projection_depth must be greater than 0.1: '
                f'{self.max_projection_depth}'
            )

        if self.projection_point_radius < 0:
            raise RuntimeError(
                'projection_point_radius must not be negative: '
                f'{self.projection_point_radius}'
            )

        if self.timestamp_policy not in ('rebase_kitti', 'ros_now'):
            raise RuntimeError(
                'timestamp_policy must be rebase_kitti or ros_now: '
                f'{self.timestamp_policy}'
            )

        if self.calibration_format not in ('kitti', 'yaml'):
            raise RuntimeError(
                'calibration_format must be kitti or yaml: '
                f'{self.calibration_format}'
            )

        if max_sensor_time_offset_sec < 0.0:
            raise RuntimeError(
                'max_sensor_time_offset_sec must not be negative: '
                f'{max_sensor_time_offset_sec}'
            )

        self.image_loader = ImageLoader(image_dir, loop=loop)
        self.pointcloud_loader = PointCloudLoader(
            pointcloud_dir,
            loop=loop,
        )
        if self.calibration_format == 'kitti':
            self.camera_info = load_camera_info(calibration_path)
            self.lidar_camera_calibration = load_lidar_camera_calibration(
                calibration_path,
                velodyne_calibration_path,
            )
        else:
            self.camera_info = load_camera_info_yaml(
                camera_intrinsics_yaml
            )
            self.lidar_camera_calibration = (
                load_lidar_camera_calibration_yaml(
                    camera_intrinsics_yaml,
                    extrinsics_yaml,
                    expected_parent_frame=self.lidar_frame_id,
                    expected_child_frame=self.camera_frame_id,
                )
            )
        self.timestamps = TimestampSequence.load(
            image_timestamps_path,
            pointcloud_timestamps_path,
        )

        if len(self.image_loader) != len(self.pointcloud_loader):
            raise RuntimeError(
                'Image and point cloud frame counts do not match: '
                f'images={len(self.image_loader)}, '
                f'pointclouds={len(self.pointcloud_loader)}'
            )

        if len(self.image_loader) != len(self.timestamps):
            raise RuntimeError(
                'Dataset and timestamp frame counts do not match: '
                f'frames={len(self.image_loader)}, '
                f'timestamps={len(self.timestamps)}'
            )

        maximum_offset_sec = (
            self.timestamps.max_absolute_sensor_offset_ns / 1e9
        )
        if maximum_offset_sec > max_sensor_time_offset_sec:
            raise RuntimeError(
                'KITTI camera-LiDAR timestamp offset exceeds limit: '
                f'maximum={maximum_offset_sec:.6f}s, '
                f'limit={max_sensor_time_offset_sec:.6f}s'
            )

        self.image_publisher = self.create_publisher(
            Image,
            '/kitti/camera/image_raw',
            10,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            '/kitti/camera/camera_info',
            10,
        )
        reliable_sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/kitti/velodyne/points',
            reliable_sensor_qos,
        )
        self.overlay_publisher = self.create_publisher(
            Image,
            '/kitti/camera/lidar_overlay',
            reliable_sensor_qos,
        )

        self.bridge = CvBridge()
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        self.published_frame_count = 0
        self.ros_start_ns = self.get_clock().now().nanoseconds

        self.publish_period_sec = 1.0 / publish_rate
        self.timer = None
        self.startup_timer = None
        if startup_delay_sec > 0.0:
            self.startup_timer = self.create_timer(
                startup_delay_sec,
                self.start_publishing,
            )
        else:
            self.start_publishing()

        self.get_logger().info(
            f'Publishing {len(self.image_loader)} synchronized KITTI '
            f'image/point-cloud frames at {publish_rate:.1f} Hz '
            f'(loop={loop}, calibration={self.calibration_format})'
        )
        if startup_delay_sec > 0.0:
            self.get_logger().info(
                f'First frame delayed by {startup_delay_sec:.1f}s '
                'for subscriber discovery'
            )
        self.get_logger().info(
            'Timestamp policy: '
            f'{self.timestamp_policy}; maximum original sensor offset: '
            f'{maximum_offset_sec * 1000.0:.3f} ms'
        )

    def start_publishing(self):
        if self.startup_timer is not None:
            self.startup_timer.cancel()
        if self.timer is None:
            self.timer = self.create_timer(
                self.publish_period_sec,
                self.publish_image,
            )

    def publish_static_transform(self):
        # KITTI calibration maps Velodyne coordinates into the rectified
        # camera coordinate system. TF stores the child pose in its parent,
        # so invert it for parent=velodyne, child=camera_optical_frame.
        camera_from_lidar = (
            self.lidar_camera_calibration.r_rect_00
            @ self.lidar_camera_calibration.tr_velo_to_cam
        )
        lidar_from_camera = np.linalg.inv(camera_from_lidar)
        quaternion = matrix_to_quaternion(lidar_from_camera[:3, :3])

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.lidar_frame_id
        transform.child_frame_id = self.camera_frame_id
        transform.transform.translation.x = lidar_from_camera[0, 3]
        transform.transform.translation.y = lidar_from_camera[1, 3]
        transform.transform.translation.z = lidar_from_camera[2, 3]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.static_transform_broadcaster.sendTransform(transform)

    def make_stamp(self, frame_index):
        if self.timestamp_policy == 'ros_now':
            return self.get_clock().now().to_msg()

        cycle_index = self.published_frame_count // len(self.timestamps)
        stamp_ns = self.timestamps.rebased_timestamp_ns(
            frame_index,
            cycle_index,
            self.ros_start_ns,
        )
        return Time(nanoseconds=stamp_ns).to_msg()

    def publish_image(self):
        frame_index = self.published_frame_count % len(self.image_loader)

        try:
            cv_image, image_path = self.image_loader.next_image()
            points, pointcloud_path = (
                self.pointcloud_loader.next_pointcloud()
            )
        except StopIteration:
            self.timer.cancel()
            self.get_logger().info('Finished publishing KITTI image sequence')
            return

        if image_path.stem != pointcloud_path.stem:
            raise RuntimeError(
                'Image and point cloud frame IDs do not match: '
                f'image={image_path.name}, '
                f'pointcloud={pointcloud_path.name}'
            )

        image_height, image_width = cv_image.shape[:2]

        if (
            image_width != self.camera_info.width
            or image_height != self.camera_info.height
        ):
            raise RuntimeError(
                'Image size does not match calibration: '
                f'image={image_width}x{image_height}, '
                f'calibration={self.camera_info.width}x'
                f'{self.camera_info.height}'
            )

        message = self.bridge.cv2_to_imgmsg(
            cv_image,
            encoding='bgr8',
        )

        stamp = self.make_stamp(frame_index)

        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame_id
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.camera_frame_id
        pointcloud_message = create_pointcloud2(
            points,
            stamp,
            frame_id=self.lidar_frame_id,
        )

        overlay_message = None

        if self.overlay_publisher.get_subscription_count() > 0:
            pixels, depths, _ = project_velodyne_to_image(
                points,
                self.lidar_camera_calibration,
                image_width,
                image_height,
                min_depth=0.1,
                max_depth=self.max_projection_depth,
            )
            overlay = draw_lidar_overlay(
                cv_image,
                pixels,
                depths,
                max_depth=self.max_projection_depth,
                point_radius=self.projection_point_radius,
            )
            overlay_message = self.bridge.cv2_to_imgmsg(
                overlay,
                encoding='bgr8',
            )
            overlay_message.header.stamp = stamp
            overlay_message.header.frame_id = self.camera_frame_id

        self.image_publisher.publish(message)
        self.camera_info_publisher.publish(self.camera_info)
        self.pointcloud_publisher.publish(pointcloud_message)

        if overlay_message is not None:
            self.overlay_publisher.publish(overlay_message)

        self.published_frame_count += 1

        self.get_logger().debug(
            f'Published KITTI frame {image_path.stem}: '
            f'{len(points)} Velodyne points'
        )


def main(args=None):
    rclpy.init(args=args)

    node = KittiPlayerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # Launch may forward SIGINT while rclpy is already tearing down.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
