"""
Launch the complete AMR demo, or the canonical single-goal benchmark.

Default (mission:=true), everything the robot does in one command:
    MuJoCo viewer, RViz, a live camera window, camera-LiDAR perception and
    fusion, Nav2 with docking, and the M01-M04 warehouse missions started
    automatically (fusion_bringup/bringup_sim.launch.py).

mission:=false keeps the canonical Gate 11/12 path used by the benchmarks:
    stage3 navigation plus perception, no mission and no scenario actors, so
    an external probe owns the Nav2 goal.

Usage:
    ros2 launch fusion_bringup final_demo.launch.py
    ros2 launch fusion_bringup final_demo.launch.py mission:=false \
        use_rviz:=false sim_gui:=false camera_view:=false
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start simulation, navigation, live perception, fusion and metrics."""
    navigation_share = Path(
        get_package_share_directory('navigation_bringup'))
    fusion_share = Path(get_package_share_directory('fusion_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    sim_gui = LaunchConfiguration('sim_gui')
    mission = LaunchConfiguration('mission')

    # Keep GTK/GIO paths from a snap-packaged editor out of native GUI tools.
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

    full_workflow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'bringup_sim.launch.py')),
        condition=IfCondition(mission),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'sim_gui': sim_gui,
            'mission_autostart': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'stage3.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'sim_gui': sim_gui,
        }.items(),
    )

    sensor_sync = Node(
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
    )
    detector = Node(
        package='yolo_detector',
        executable='simulation_color_detector',
        name='simulation_color_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    fusion = Node(
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
    )
    metrics = Node(
        package='perception_core',
        executable='metrics_node',
        name='metrics_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_period_sec': 0.5,
        }],
    )
    benchmark_path = GroupAction(
        condition=UnlessCondition(mission),
        actions=[navigation, sensor_sync, detector, fusion, metrics])

    camera_view = Node(
        package='yolo_detector',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera_view')),
        additional_env=gui_environment,
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': LaunchConfiguration('camera_topic'),
            'save_directory': LaunchConfiguration('camera_save_dir'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'sim_gui', default_value='true',
            description='Open the MuJoCo viewer window.'),
        DeclareLaunchArgument(
            'mission', default_value='true',
            description='Run the M01-M04 missions (false: canonical benchmark path).'),
        DeclareLaunchArgument(
            'camera_view', default_value='true',
            description='Open a window with the robot camera and detection boxes.'),
        DeclareLaunchArgument(
            'camera_topic', default_value='/camera/image_raw',
            description='Image shown in the camera window.'),
        DeclareLaunchArgument(
            'camera_save_dir', default_value=str(Path.home() / 'camera_dataset'),
            description="Where the camera window's s key saves raw frames."),
        full_workflow,
        benchmark_path,
        camera_view,
    ])
