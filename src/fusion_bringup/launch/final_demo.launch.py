"""Launch the canonical camera-LiDAR-to-Nav2 end-to-end demo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start simulation, navigation, live perception, fusion and metrics."""
    navigation_share = Path(
        get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'stage3.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'gazebo_gui': gazebo_gui,
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        navigation,
        sensor_sync,
        detector,
        fusion,
        metrics,
    ])
