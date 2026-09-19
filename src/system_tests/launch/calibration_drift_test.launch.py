"""Launch the calibration monitor for drift-injection experiments."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fusion_bringup'))
    config = share / 'config'
    candidate_extrinsics_path = LaunchConfiguration(
        'candidate_extrinsics_path'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'candidate_extrinsics_path',
            default_value=str(config / 'lidar_camera_extrinsics.yaml'),
            description='Extrinsic YAML to monitor for injected drift.',
        ),
        Node(
            package='perception_core',
            executable='calibration_monitor_node',
            output='screen',
            parameters=[
                str(config / 'calibration_monitor_params.yaml'),
                {
                    'intrinsics_path': str(
                        config / 'camera_intrinsics.yaml'
                    ),
                    'reference_extrinsics_path': str(
                        config / 'lidar_camera_extrinsics.yaml'
                    ),
                    'candidate_extrinsics_path': candidate_extrinsics_path,
                },
            ],
        ),
    ])
