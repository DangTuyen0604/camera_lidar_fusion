"""
Launch map servers, AMCL and their lifecycle manager.

AMCL localizes against its own map (only geometry in the LiDAR scan plane,
finer cells, half-cell origin correction; see tools/generate_warehouse_map.py),
while /map keeps every obstacle for the planner and costmaps.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_bringup'))
    map_file = LaunchConfiguration('map')
    localization_map = LaunchConfiguration('localization_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps' / 'warehouse_map.yaml')),
        DeclareLaunchArgument(
            'localization_map',
            default_value=str(share / 'maps' / 'warehouse_localization_map.yaml'),
            description='Map AMCL matches scans against; pass the same file as '
                        'map:= when using a map without a localization variant.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config' / 'nav2_params.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params_file, {
                 'yaml_filename': map_file, 'use_sim_time': use_sim_time}]),
        Node(package='nav2_map_server', executable='map_server',
             name='localization_map_server', output='screen', parameters=[{
                 'yaml_filename': localization_map, 'topic_name': 'localization_map',
                 'frame_id': 'map', 'use_sim_time': use_sim_time}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': use_sim_time, 'autostart': autostart,
                 'node_names': ['map_server', 'localization_map_server', 'amcl']}]),
    ])
