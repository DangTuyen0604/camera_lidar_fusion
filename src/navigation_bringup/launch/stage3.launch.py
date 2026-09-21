"""Launch Stage 3 localization, Nav2, velocity safety, and RViz."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    """Include one launch file from an installed package."""
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    """Start the Stage 2 baseline plus the Stage 3 safety command chain."""
    share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    gui_environment = {
        'GIO_MODULE_DIR': '',
        'GTK_EXE_PREFIX': '',
        'GTK_IM_MODULE_FILE': '',
        'GTK_PATH': '',
        'XDG_DATA_HOME': os.environ.get('XDG_DATA_HOME_VSCODE_SNAP_ORIG', ''),
        'XDG_DATA_DIRS': os.environ.get(
            'XDG_DATA_DIRS_VSCODE_SNAP_ORIG', '/usr/local/share:/usr/share'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=str(share / 'maps' / 'warehouse_slam_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'nav2_stage3_params.yaml'),
        ),
        include('warehouse_simulation', 'warehouse.launch.py', {
            'gui': LaunchConfiguration('gazebo_gui'),
            'use_sim_time': use_sim_time,
            'use_scenario': 'false',
        }),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_body_filter',
            output='screen',
            parameters=[str(share / 'config' / 'scan_body_filter.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('scan', '/scan'),
                        ('scan_filtered', '/scan_filtered')],
        ),
        TimerAction(period=3.0, actions=[
            include('navigation_bringup', 'localization.launch.py', {
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': params_file,
            }),
        ]),
        TimerAction(period=5.0, actions=[
            include('navigation_bringup', 'navigation_stage3.launch.py', {
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }),
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', str(share / 'rviz' / 'navigation_stage2.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                additional_env=gui_environment, output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
