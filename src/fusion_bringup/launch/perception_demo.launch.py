"""Run KITTI playback/projection together with YOLO fusion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_share = Path(get_package_share_directory('fusion_bringup'))
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(fusion_share / 'launch' / 'projection_demo.launch.py')
            ),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),
        Node(
            package='yolo_detector',
            executable='yolo_detector',
            name='yolo_detector_node',
            output='screen',
        ),
        Node(
            package='yolo_detector',
            executable='fusion_node',
            name='camera_lidar_fusion_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(fusion_share / 'rviz' / 'perception.rviz')],
            additional_env={'GTK_PATH': ''},
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
