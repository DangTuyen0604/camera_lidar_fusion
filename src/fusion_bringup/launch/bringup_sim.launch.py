# SPDX-License-Identifier: Apache-2.0
"""
One-command bringup for the integrated warehouse perception stack.

Starts one coherent simulation-time pipeline:

    1. Gazebo warehouse, robot, sensor bridges and scenario runner
    2. live camera-LiDAR synchronization, detection and XYZ fusion
    3. obstacle bridge, localization, Nav2 and collision safety
    4. RViz and the M01-M04 warehouse mission

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
                # ScenarioRunner still owns physical objects and benchmark
                # truth, but only live perception may drive the bridge.
                'publish_fused_detections': 'false',
            }.items(),
        ),
        Node(
            package='perception_core',
            executable='sensor_sync_node',
            name='sensor_sync_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'pointcloud_topic': '/lidar/points',
                'camera_frame_id': 'camera_optical_frame',
                'lidar_frame_id': 'lidar_link',
                'sync_tolerance_ms': 120.0,
            }],
        ),
        Node(
            package='yolo_detector',
            executable='simulation_color_detector',
            name='simulation_color_detector_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='perception_core',
            executable='object_fusion_node',
            name='object_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_frame_id': 'camera_optical_frame',
                'lidar_frame_id': 'lidar_link',
                'sync_tolerance_ms': 120.0,
                'tf_timeout_ms': 100.0,
                'min_lidar_points': 1,
                'bbox_shrink_ratio': 0.90,
                'min_depth': 0.40,
                'max_depth': 10.0,
            }],
        ),
        Node(
            package='perception_core',
            executable='metrics_node',
            name='metrics_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_period_sec': 0.5,
            }],
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
