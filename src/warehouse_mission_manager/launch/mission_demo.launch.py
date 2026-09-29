from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    share = Path(get_package_share_directory('warehouse_mission_manager'))
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'missions_file', default_value=str(share / 'config' / 'missions.yaml')),
        Node(package='warehouse_mission_manager', executable='mission_manager',
             name='warehouse_mission_manager', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': LaunchConfiguration('autostart'),
                          'missions_file': LaunchConfiguration('missions_file')}]),
    ])
