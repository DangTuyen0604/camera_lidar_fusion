"""Launch the complete project: simulation, Nav2, perception and mission."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments):
    """Include a launch file from an installed package."""
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=arguments.items(),
    )


def workspace_root():
    """Return the colcon workspace root for merged or isolated installs."""
    prefix = Path(get_package_prefix('fusion_bringup'))
    install_dir = prefix if prefix.name == 'install' else prefix.parent
    return install_dir.parent


def generate_launch_description():
    """Compose all currently available project demonstrations."""
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    mission_autostart = LaunchConfiguration('mission_autostart')
    dataset_root = LaunchConfiguration('dataset_root')
    model_path = LaunchConfiguration('model_path')
    loop = LaunchConfiguration('loop')

    root = workspace_root()
    default_dataset = str(root / 'data' / 'kitti' / '2011_09_26')
    default_model = str(root / 'models' / 'yolov8n-opencv.onnx')

    actions = []
    venv_site = (
        root / '.venv' / 'lib' /
        f'python{sys.version_info.major}.{sys.version_info.minor}' /
        'site-packages'
    )
    if venv_site.is_dir():
        python_path = str(venv_site)
        if os.environ.get('PYTHONPATH'):
            python_path += os.pathsep + os.environ['PYTHONPATH']
        actions.append(SetEnvironmentVariable('PYTHONPATH', python_path))

    actions.extend([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mission_autostart', default_value='true'),
        DeclareLaunchArgument('dataset_root', default_value=default_dataset),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('loop', default_value='true'),
        include('navigation_bringup', 'warehouse_full_demo.launch.py', {
            'gui': gui,
            'mission_autostart': mission_autostart,
        }),
        include('fusion_bringup', 'perception_demo.launch.py', {
            'dataset_root': dataset_root,
            'model_path': model_path,
            'use_rviz': use_rviz,
            'loop': loop,
        }),
    ])
    return LaunchDescription(actions)
