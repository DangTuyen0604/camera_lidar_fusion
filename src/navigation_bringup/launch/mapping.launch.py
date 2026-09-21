"""Launch online asynchronous SLAM for the demo robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            str(share / 'config' / 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_body_filter',
            output='screen',
            parameters=[
                str(share / 'config' / 'scan_body_filter.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('scan', '/scan'),
                        ('scan_filtered', '/scan_filtered')],
        ),
        slam_toolbox,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
        Node(
            package='nav2_map_server', executable='map_saver_server',
            name='map_saver', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'save_map_timeout': 5.0,
                'free_thresh_default': 0.25,
                'occupied_thresh_default': 0.65,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map_saver', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_saver'],
            }],
        ),
    ])
