"""One-command warehouse simulation, localization, Nav2, fusion bridge and M01-M04."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    nav_share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    map_file = LaunchConfiguration('map')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument(
            'map', default_value=str(nav_share / 'maps' / 'warehouse_map.yaml')),
        include('warehouse_simulation', 'warehouse.launch.py', {
            'gui': gui, 'use_scenario': 'true'}),
        Node(package='navigation_bridge', executable='detection_obstacle_bridge_node',
             output='screen', parameters=[{
                 'use_sim_time': use_sim_time, 'target_frame': 'base_link',
                 'minimum_confidence': 0.35, 'minimum_range': 0.20,
                 'maximum_range': 30.0, 'obstacle_timeout': 0.75}]),
        TimerAction(period=3.0, actions=[
            include('navigation_bringup', 'localization.launch.py', {
                'map': map_file, 'use_sim_time': use_sim_time}),
            include('navigation_bringup', 'navigation.launch.py', {
                'use_sim_time': use_sim_time}),
        ]),
        TimerAction(period=8.0, actions=[
            include('warehouse_mission_manager', 'mission_demo.launch.py'),
        ]),
    ])
