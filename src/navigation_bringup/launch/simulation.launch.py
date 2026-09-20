"""Launch the canonical robot in the warehouse simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    warehouse = Path(get_package_share_directory('warehouse_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_scenario', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(warehouse / 'launch' / 'warehouse.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'use_scenario': LaunchConfiguration('use_scenario'),
            }.items()),
    ])
