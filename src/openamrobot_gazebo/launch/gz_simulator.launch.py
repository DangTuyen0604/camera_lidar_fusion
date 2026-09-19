import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('openamrobot_description')
    gazebo_dir = get_package_share_directory('openamrobot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    xacro_file = os.path.join(description_dir, 'urdf', 'robo_urdf.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    resource_paths = [
        os.path.join(gazebo_dir, 'worlds'),
        os.path.dirname(description_dir),
    ]

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths),
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        condition=IfCondition(use_robot_state_pub),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -v 2 ', world]}.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ['-r -s -v 2 ', world]}.items(),
        condition=UnlessCondition(gui),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'openamrobot',
            '-topic', '/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'use_robot_state_pub', default_value='True',
            description='Whether to start robot_state_publisher'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                gazebo_dir, 'worlds', 'mobile_robot_world.sdf'
            ),
            description='Full path to the Gazebo world file (.sdf) to load'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start Gazebo GUI. Default false keeps simulation timing stable.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.20'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        gz_resource_path,
        gz_sim_gui,
        gz_sim_headless,
        bridge,
        spawn_entity,
        start_robot_state_publisher_cmd,
    ])
