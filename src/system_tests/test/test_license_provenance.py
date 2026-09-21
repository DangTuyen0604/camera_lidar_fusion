"""License declarations must match project ownership and provenance."""

from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_LICENSES = {
    'fusion_bringup': 'Apache-2.0',
    'fusion_interfaces': 'Apache-2.0',
    'kitti_ros2_player': 'Apache-2.0',
    'navigation_bridge': 'Apache-2.0',
    'navigation_bringup': 'Apache-2.0',
    'openamrobot_description': 'MIT',
    'openamrobot_gazebo': 'MIT',
    'perception_core': 'Apache-2.0',
    'system_tests': 'Apache-2.0',
    'warehouse_mission_manager': 'Apache-2.0',
    'warehouse_simulation': 'Apache-2.0',
    'yolo_detector': 'Apache-2.0',
}


def test_package_licenses_match_provenance():
    discovered = {}
    for package_xml in (REPO_ROOT / 'src').rglob('package.xml'):
        root = ET.parse(package_xml).getroot()
        discovered[root.findtext('name').strip()] = root.findtext('license').strip()
    assert discovered == EXPECTED_LICENSES


def test_openamrobot_notice_and_licenses_are_shipped():
    notice = (REPO_ROOT / 'THIRD_PARTY_NOTICES.md').read_text(encoding='utf-8')
    assert 'openAMRobot/openamr-platform-sw' in notice
    assert 'MIT' in notice
    assert 'CERN-OHL-P-2.0' in notice
    assert '13c76cce0b8c907e2c578fc281ccf37b7b01342b' in notice
    for package in ('openamrobot_description', 'openamrobot_gazebo'):
        package_root = REPO_ROOT / 'src' / package
        assert (package_root / 'LICENSE').is_file()
        assert (package_root / 'LICENSING.md').is_file()
