from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='warehouse_mission_manager', executable='mission_manager',
             name='warehouse_mission_manager', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': LaunchConfiguration('autostart')}]),
    ])
