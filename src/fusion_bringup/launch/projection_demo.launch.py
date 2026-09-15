from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    dataset_root = LaunchConfiguration('dataset_root')
    use_rviz = LaunchConfiguration('use_rviz')
    loop = LaunchConfiguration('loop')
    publish_rate = LaunchConfiguration('publish_rate')
    timestamp_policy = LaunchConfiguration('timestamp_policy')
    calibration_format = LaunchConfiguration('calibration_format')
    camera_intrinsics_yaml = LaunchConfiguration('camera_intrinsics_yaml')
    extrinsics_yaml = LaunchConfiguration('extrinsics_yaml')

    default_dataset_root = str(
        Path.cwd() / 'data' / 'kitti' / '2011_09_26'
    )
    sequence_root = PathJoinSubstitution([
        dataset_root,
        '2011_09_26_drive_0005_sync',
    ])
    rviz_config = str(
        Path(get_package_share_directory('fusion_bringup'))
        / 'rviz'
        / 'perception.rviz'
    )
    calibration_config_dir = Path(
        get_package_share_directory('fusion_bringup')
    ) / 'config'
    sync_parameters = str(calibration_config_dir / 'sync_params.yaml')

    player = Node(
        package='kitti_ros2_player',
        executable='kitti_player',
        name='kitti_player',
        output='screen',
        parameters=[{
            'image_dir': PathJoinSubstitution([
                sequence_root, 'image_02', 'data',
            ]),
            'pointcloud_dir': PathJoinSubstitution([
                sequence_root, 'velodyne_points', 'data',
            ]),
            'image_timestamps_path': PathJoinSubstitution([
                sequence_root, 'image_02', 'timestamps.txt',
            ]),
            'pointcloud_timestamps_path': PathJoinSubstitution([
                sequence_root, 'velodyne_points', 'timestamps.txt',
            ]),
            'calibration_path': PathJoinSubstitution([
                dataset_root, 'calib_cam_to_cam.txt',
            ]),
            'velodyne_calibration_path': PathJoinSubstitution([
                dataset_root, 'calib_velo_to_cam.txt',
            ]),
            'loop': loop,
            'publish_rate': publish_rate,
            'timestamp_policy': timestamp_policy,
            'calibration_format': calibration_format,
            'camera_intrinsics_yaml': camera_intrinsics_yaml,
            'extrinsics_yaml': extrinsics_yaml,
        }],
    )

    sensor_sync = Node(
        package='perception_core',
        executable='sensor_sync_node',
        name='sensor_sync_node',
        output='screen',
        parameters=[sync_parameters],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        additional_env={'GTK_PATH': ''},
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_root',
            default_value=default_dataset_root,
            description='KITTI date directory containing calibration files.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'timestamp_policy',
            default_value='rebase_kitti',
            description='rebase_kitti or ros_now',
        ),
        DeclareLaunchArgument(
            'calibration_format',
            default_value='kitti',
            description='kitti or yaml',
        ),
        DeclareLaunchArgument(
            'camera_intrinsics_yaml',
            default_value=str(
                calibration_config_dir / 'camera_intrinsics.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'extrinsics_yaml',
            default_value=str(
                calibration_config_dir / 'lidar_camera_extrinsics.yaml'
            ),
        ),
        player,
        sensor_sync,
        rviz,
    ])
