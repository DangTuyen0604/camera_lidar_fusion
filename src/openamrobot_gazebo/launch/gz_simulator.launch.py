import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_sim.actions import GzServer


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

    xacro_file = os.path.join(description_dir, 'urdf', 'mobile_robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    resource_paths = [
        os.path.join(gazebo_dir, 'worlds'),
        os.path.dirname(description_dir),
    ]

    # VS Code installed through snap exports GTK / GIO search paths from the
    # core20 runtime.  Native Jazzy GUI processes then load snap's libpthread
    # and fail before creating a window.  Keep those paths out of Gazebo while
    # leaving the ROS and Gazebo vendor library paths intact.
    gui_environment = {
        'GIO_MODULE_DIR': '',
        'GTK_EXE_PREFIX': '',
        'GTK_IM_MODULE_FILE': '',
        'GTK_PATH': '',
        'XDG_DATA_HOME': os.environ.get(
            'XDG_DATA_HOME_VSCODE_SNAP_ORIG', ''),
        'XDG_DATA_DIRS': os.environ.get(
            'XDG_DATA_DIRS_VSCODE_SNAP_ORIG',
            '/usr/local/share:/usr/share'),
    }

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
               ':', ':'.join(resource_paths)],
    )
    # Isolate each launch from stale or unrelated Gazebo Transport publishers.
    # Without this, a recently stopped simulation can make the next server
    # namespace /clock and /stats, which also prevents /odom and /scan from
    # reaching the ROS bridge.
    gz_partition = SetEnvironmentVariable(
        name='GZ_PARTITION', value=f'camera_lidar_fusion_{os.getpid()}')

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

    # GzServer embeds Gazebo and exposes the ROS spawn/delete/set-pose
    # services used by the deterministic scenario runner.
    gz_server = GzServer(world_sdf_file=world, verbosity_level=2)
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '2'],
        condition=IfCondition(gui), output='screen',
        additional_env=gui_environment)

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
        gz_partition,
        gz_server,
        gz_gui,
        bridge,
        spawn_entity,
        start_robot_state_publisher_cmd,
    ])
