# SPDX-License-Identifier: Apache-2.0
"""
One-command bringup for the complete warehouse simulation stack.

Starts one coherent simulation-time pipeline:

    1. Gazebo warehouse, robot, sensor bridges and scenario runner
    2. localization and Nav2
    3. RViz
    4. warehouse mission manager with mission autostart

Usage:
    ros2 launch fusion_bringup bringup_sim.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo, Nav2, RViz and the autonomous warehouse mission."""
    nav_share = Path(get_package_share_directory('navigation_bringup'))
    warehouse_launch = nav_share / 'launch' / 'warehouse_full_demo.launch.py'

    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    mission_autostart = LaunchConfiguration('mission_autostart')
    rviz_delay = LaunchConfiguration('rviz_delay')

    # Do not let GTK/GIO paths inherited from a snap-packaged editor inject
    # core20 libraries into the native RViz process.
    gui_environment = {
        'GIO_MODULE_DIR': '',
        'GTK_EXE_PREFIX': '',
        'GTK_IM_MODULE_FILE': '',
        'GTK_PATH': '',
        'XDG_DATA_HOME': os.environ.get(
            'XDG_DATA_HOME_VSCODE_SNAP_ORIG', ''),
        'XDG_DATA_DIRS': os.environ.get(
            'XDG_DATA_DIRS_VSCODE_SNAP_ORIG',
            '/usr/local/share:/usr/share'),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Start the Gazebo graphical interface.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock throughout the stack.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the navigation view.'),
        DeclareLaunchArgument(
            'mission_autostart', default_value='true',
            description='Start the warehouse mission automatically.'),
        DeclareLaunchArgument(
            'rviz_delay', default_value='5.0',
            description='Seconds to wait before starting RViz.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(warehouse_launch)),
            launch_arguments={
                'gui': gazebo_gui,
                'use_sim_time': use_sim_time,
                'mission_autostart': mission_autostart,
            }.items(),
        ),
        TimerAction(period=rviz_delay, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', str(nav_share / 'rviz' / 'navigation.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                additional_env=gui_environment,
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
