"""Launch Nav2 with one safety-enforced velocity path to Gazebo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    managed_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]
    common = [params_file, {'use_sim_time': use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'nav2_params.yaml'),
        ),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common),
        Node(package='nav2_route', executable='route_server',
             name='route_server', output='screen', parameters=common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=common),
        # Docking must enter the same smoother/safety chain as navigation.
        # Publishing directly to /cmd_vel bypasses Collision Monitor.
        Node(package='opennav_docking', executable='opennav_docking',
             name='docking_server', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'autostart': autostart,
                 'node_names': managed_nodes,
             }]),
    ])
