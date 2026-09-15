"""Launch localization, Nav2, robot state, and perception obstacle bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        str(share / 'urdf' / 'mobile_robot.urdf.xacro'),
    ])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'localization.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps' / 'demo_map.yaml')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='navigation_bridge',
            executable='detection_obstacle_bridge_node',
            output='screen',
            parameters=[{
                'minimum_confidence': 0.35,
                'maximum_range': 50.0,
            }],
        ),
        localization,
        navigation,
    ])
