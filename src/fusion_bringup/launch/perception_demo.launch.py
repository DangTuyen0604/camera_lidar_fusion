"""Run the synchronized KITTI, YOLO and typed 3D fusion pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fusion_share = Path(get_package_share_directory('fusion_bringup'))
    config_dir = fusion_share / 'config'

    dataset_root = LaunchConfiguration('dataset_root')
    model_path = LaunchConfiguration('model_path')
    use_rviz = LaunchConfiguration('use_rviz')
    loop = LaunchConfiguration('loop')
    publish_rate = LaunchConfiguration('publish_rate')
    timestamp_policy = LaunchConfiguration('timestamp_policy')
    calibration_format = LaunchConfiguration('calibration_format')

    default_dataset_root = str(
        Path.cwd() / 'data' / 'kitti' / '2011_09_26'
    )
    default_model_path = str(Path.cwd() / 'models' / 'yolov8n-opencv.onnx')

    projection_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(fusion_share / 'launch' / 'projection_demo.launch.py')
        ),
        launch_arguments={
            'dataset_root': dataset_root,
            'use_rviz': 'false',
            'loop': loop,
            'publish_rate': publish_rate,
            'timestamp_policy': timestamp_policy,
            'calibration_format': calibration_format,
        }.items(),
    )

    yolo = Node(
        package='yolo_detector',
        executable='yolo_detector',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            str(config_dir / 'yolo_params.yaml'),
            {'model_path': model_path},
        ],
    )

    fusion = Node(
        package='perception_core',
        executable='object_fusion_node',
        name='object_fusion_node',
        output='screen',
        parameters=[str(config_dir / 'fusion_params.yaml')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(fusion_share / 'rviz' / 'perception.rviz')],
        additional_env={'GTK_PATH': ''},
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_root',
            default_value=default_dataset_root,
            description='KITTI date directory containing calibration files.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to the exported YOLO ONNX model.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('timestamp_policy', default_value='rebase_kitti'),
        DeclareLaunchArgument('calibration_format', default_value='kitti'),
        projection_pipeline,
        yolo,
        fusion,
        rviz,
    ])
