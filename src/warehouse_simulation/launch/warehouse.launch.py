"""Start the MuJoCo warehouse in GUI/headless mode and its deterministic scenario."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('warehouse_simulation'))
    mujoco = Path(get_package_share_directory('warehouse_mujoco'))
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_scenario = LaunchConfiguration('use_scenario')
    enable_roaming = LaunchConfiguration('enable_roaming')
    random_seed = LaunchConfiguration('random_seed')
    update_period = LaunchConfiguration('update_period_sec')
    publish_fused_detections = LaunchConfiguration(
        'publish_fused_detections')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Open the MuJoCo viewer window.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_scenario', default_value='true'),
        DeclareLaunchArgument('enable_roaming', default_value='true'),
        DeclareLaunchArgument(
            'random_seed', default_value='42',
            description='Reproducible seed for independent pedestrian roaming.'),
        DeclareLaunchArgument(
            'update_period_sec', default_value='0.20',
            description='Scenario motion/truth update period in seconds.'),
        DeclareLaunchArgument(
            'publish_fused_detections', default_value='true',
            description=(
                'Publish deterministic scenario truth on the live fusion '
                'topic. Disable when the real perception pipeline is active.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mujoco / 'launch' / 'mujoco_sim.launch.py')),
            launch_arguments={
                'world': str(share / 'worlds' / 'warehouse.sdf'),
                'model_paths': str(share / 'models'),
                'gui': gui,
                'use_sim_time': use_sim_time,
                'spawn_x': '0.0',
                'spawn_y': '-4.0',
                'spawn_yaw': '0.0',
            }.items()),
        Node(package='warehouse_simulation', executable='scenario_runner',
             condition=IfCondition(use_scenario), output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'enable_roaming': enable_roaming,
                 'random_seed': random_seed,
                 'update_period_sec': update_period,
                 'publish_fused_detections': publish_fused_detections,
             }]),
    ])
