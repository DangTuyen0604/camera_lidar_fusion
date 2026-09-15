from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def test_navigation_assets_are_installed_and_parseable():
    share = Path(get_package_share_directory('navigation_bringup'))
    required = [
        share / 'launch' / 'full_system.launch.py',
        share / 'launch' / 'navigation.launch.py',
        share / 'config' / 'nav2_params.yaml',
        share / 'maps' / 'demo_map.yaml',
        share / 'urdf' / 'mobile_robot.urdf.xacro',
    ]
    assert all(path.is_file() and path.stat().st_size > 0 for path in required)
    with required[2].open(encoding='utf-8') as stream:
        assert 'controller_server' in yaml.safe_load(stream)
