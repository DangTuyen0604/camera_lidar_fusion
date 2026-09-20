"""Start the warehouse in GUI/headless mode and its deterministic scenario."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('warehouse_simulation'))
    gazebo = Path(get_package_share_directory('openamrobot_gazebo'))
    gui = LaunchConfiguration('gui')
    use_scenario = LaunchConfiguration('use_scenario')
    from launch.conditions import IfCondition
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_scenario', default_value='true'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            f"{share / 'models'}:{share / 'worlds'}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'gz_simulator.launch.py')),
            launch_arguments={
                'world': str(share / 'worlds' / 'warehouse.sdf'),
                'gui': gui, 'spawn_x': '0.0', 'spawn_y': '-4.0', 'spawn_yaw': '0.0',
            }.items()),
        Node(package='warehouse_simulation', executable='scenario_runner',
             condition=IfCondition(use_scenario), output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
