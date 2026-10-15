"""
Launch the full warehouse demo with the trained YOLO model.

final_demo.launch.py with the settings for a live run of the trained
detector: detector:=yolo, the model in models/ and the camera window showing
YOLO's boxes. Any of them can still be overridden, and every other
final_demo argument passes through (e.g. odometry:=ekf, mission:=false).

Usage:
    ros2 launch fusion_bringup yolo_demo.launch.py
    ros2 launch fusion_bringup yolo_demo.launch.py odometry:=ekf mission:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# The first model found in models/ of the source tree (the working directory
# on the host, /workspace in Docker).
MODEL_NAMES = ('best.onnx', 'warehouse_yolov8n.onnx')
MODEL_DIRS = (Path.cwd() / 'models', Path('/workspace/models'))
DEFAULT_MODEL = str(next(
    (directory / name for name in MODEL_NAMES for directory in MODEL_DIRS
     if (directory / name).is_file()),
    MODEL_DIRS[0] / MODEL_NAMES[0]))


def generate_launch_description():
    """Include final_demo with the YOLO detector and its camera view."""
    final_demo = Path(get_package_share_directory('fusion_bringup')) / 'launch' / \
        'final_demo.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path', default_value=DEFAULT_MODEL,
            description='YOLO ONNX model (default: first of models/best.onnx, '
                        'models/warehouse_yolov8n.onnx).'),
        DeclareLaunchArgument(
            'camera_topic', default_value='/yolo/debug/image',
            description='Image shown in the camera window.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(final_demo)),
            launch_arguments={
                'detector': 'yolo',
                'model_path': LaunchConfiguration('model_path'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            }.items()),
    ])
