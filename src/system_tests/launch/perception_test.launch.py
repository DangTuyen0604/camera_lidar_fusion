"""Headless perception pipeline launch used for manual system validation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = Path(get_package_share_directory('fusion_bringup'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup / 'launch' / 'perception_demo.launch.py')
            ),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),
    ])
