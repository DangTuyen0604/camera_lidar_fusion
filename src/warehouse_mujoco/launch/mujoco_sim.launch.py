"""Start the MuJoCo simulator and robot_state_publisher (replaces gz_simulator)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('openamrobot_description')
    xacro_file = os.path.join(description_dir, 'urdf', 'mobile_robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    use_sim_time = LaunchConfiguration('use_sim_time')
    arguments = [
        ('world', '', 'SDF world converted into the MuJoCo scene (empty = flat floor).'),
        ('model_paths', '', 'Colon-separated directories resolving model:// URIs.'),
        ('gui', 'false', 'Open the MuJoCo viewer window.'),
        ('use_sim_time', 'true', 'Use the simulator /clock.'),
        ('use_robot_state_pub', 'true', 'Start robot_state_publisher.'),
        ('enable_camera', 'true', 'Render /camera/image_raw (needs OpenGL).'),
        ('real_time_factor', '1.0', 'Simulation speed; 0 runs as fast as possible.'),
        ('spawn_x', '0.0', ''), ('spawn_y', '-4.0', ''), ('spawn_yaw', '0.0', ''),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=text)
          for name, value, text in arguments],
        Node(
            package='warehouse_mujoco', executable='mujoco_sim', name='mujoco_sim',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'model_paths': LaunchConfiguration('model_paths'),
                'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
                'enable_camera': ParameterValue(
                    LaunchConfiguration('enable_camera'), value_type=bool),
                'real_time_factor': ParameterValue(
                    LaunchConfiguration('real_time_factor'), value_type=float),
                'spawn_x': ParameterValue(LaunchConfiguration('spawn_x'), value_type=float),
                'spawn_y': ParameterValue(LaunchConfiguration('spawn_y'), value_type=float),
                'spawn_yaw': ParameterValue(LaunchConfiguration('spawn_yaw'), value_type=float),
                'use_sim_time': False,
            }]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='both',
            condition=IfCondition(LaunchConfiguration('use_robot_state_pub')),
            parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_desc}]),
    ])
