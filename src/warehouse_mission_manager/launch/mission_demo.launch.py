from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(package='warehouse_mission_manager', executable='mission_manager',
             name='warehouse_mission_manager', output='screen',
             parameters=[{'use_sim_time': True,
                          'autostart': LaunchConfiguration('autostart')}]),
    ])
