"""
Run the C++ LiDAR preprocessing node with a per-sensor parameter profile.

sensor_profile picks perception_core's config/<name>.yaml (kitti_hdl64,
sim_planar, sim_vlp16, custom_lidar, or your own copy); params_file
overrides it with any path. use_composition loads the node into a component
container with intra-process communication enabled.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _nodes(context):
    share = Path(get_package_share_directory('perception_core'))
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        profile = LaunchConfiguration('sensor_profile').perform(context)
        params_file = str(share / 'config' / f'{profile}.yaml')
    if not Path(params_file).is_file():
        raise RuntimeError(f'pointcloud_preprocess profile not found: {params_file}')
    if LaunchConfiguration('use_composition').perform(context) == 'true':
        return [ComposableNodeContainer(
            name='pointcloud_preprocess_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[ComposableNode(
                package='perception_core',
                plugin='perception_core::PointCloudPreprocessNode',
                name='pointcloud_preprocess',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}])])]
    return [Node(
        package='perception_core', executable='pointcloud_preprocess_node',
        name='pointcloud_preprocess', output='screen', parameters=[params_file])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_profile', default_value='sim_planar'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_composition', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
