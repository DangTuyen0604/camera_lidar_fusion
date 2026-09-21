"""Launch the Stage 3 Nav2 stack with velocity safety enforcement."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start Nav2 with smoothing followed by collision monitoring."""
    share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    managed_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'nav2_stage3_params.yaml'),
        ),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'autostart': autostart,
                 'node_names': managed_nodes,
             }]),
    ])
