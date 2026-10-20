"""Run live perception metrics beside the complete headless warehouse stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items())


def generate_launch_description():
    mission_autostart = LaunchConfiguration('mission_autostart')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('mission_autostart', default_value='false'),
        include('navigation_bringup', 'warehouse_full_demo.launch.py', {
            'gui': LaunchConfiguration('gui'),
            'mission_autostart': mission_autostart}),
        include('fusion_bringup', 'perception_demo.launch.py', {
            'use_rviz': LaunchConfiguration('use_rviz'), 'loop': 'true'}),
    ])
