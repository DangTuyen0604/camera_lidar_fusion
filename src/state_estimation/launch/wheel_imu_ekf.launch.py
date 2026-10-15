"""Start the wheel/IMU EKF that publishes /odom and odom -> base_footprint."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('state_estimation'), 'config', 'wheel_imu_ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Not 'params_file': launch arguments are global, and that name would
        # replace Nav2's parameter file when this launch is included.
        DeclareLaunchArgument('ekf_params_file', default_value=config),
        DeclareLaunchArgument(
            'use_accelerometer', default_value='true',
            description='false: wheels + gyro only (comparison setup).'),
        Node(
            package='state_estimation', executable='wheel_imu_ekf_node',
            name='wheel_imu_ekf', output='screen',
            parameters=[
                LaunchConfiguration('ekf_params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                 'use_accelerometer': ParameterValue(
                     LaunchConfiguration('use_accelerometer'), value_type=bool)}]),
    ])
