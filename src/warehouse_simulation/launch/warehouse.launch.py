"""
Start the MuJoCo warehouse in GUI/headless mode and its deterministic scenario.

odometry selects what owns /odom and odom -> base_footprint:
  wheel         the simulator's wheel odometry (encoders only)
  ekf           state_estimation's wheel + IMU filter; wheel odometry on /wheel/odom
  ekf_no_accel  the same filter without the accelerometer (wheels + gyro)
On the robot the filter belongs to the robot bringup; here the simulator
stands in for the robot, so both start together.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    odometry = LaunchConfiguration('odometry')
    filtered = PythonExpression(["'", odometry, "' != 'wheel'"])
    return LaunchDescription([
        DeclareLaunchArgument(
            'odometry', default_value='wheel',
            choices=['wheel', 'ekf', 'ekf_no_accel'],
            description='Source of /odom: wheel odometry or the wheel + IMU EKF.'),
        DeclareLaunchArgument(
            'sensor_errors', default_value='ideal', choices=['ideal', 'realistic'],
            description="realistic: the robot's MPU-6500 IMU and AS5040 encoders, with errors."),
        DeclareLaunchArgument(
            'sim_seed', default_value='0',
            description='Seed of the simulated sensor noise and error draws.'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Open the MuJoCo viewer window.'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot heading at (0, -4); 1.57 is docked at HOME.'),
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
            'lidar_3d_channels', default_value='16',
            description='Rings of the simulated multi-layer LiDAR (0 = planar only).'),
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
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'lidar_3d_channels': LaunchConfiguration('lidar_3d_channels'),
                'odom_topic': PythonExpression(
                    ["'/wheel/odom' if ", filtered, " else '/odom'"]),
                'publish_odom_tf': PythonExpression(['not ', filtered]),
                'sensor_errors': LaunchConfiguration('sensor_errors'),
                'seed': LaunchConfiguration('sim_seed'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('state_estimation')) / 'launch' /
                'wheel_imu_ekf.launch.py')),
            condition=LaunchConfigurationNotEquals('odometry', 'wheel'),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_accelerometer': PythonExpression(
                    ["'", odometry, "' == 'ekf'"]),
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
