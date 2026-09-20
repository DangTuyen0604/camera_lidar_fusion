"""Rosbag record/replay inputs cover the perception contract."""

from pathlib import Path


def test_record_script_contains_replay_required_topics():
    root = Path(__file__).resolve().parents[3]
    script = (root / 'scripts' / 'record_personal_bag.sh').read_text()
    for topic in ('image_raw', 'camera_info', 'points', '/tf_static'):
        assert topic in script
    assert (root / 'bags' / 'README.md').is_file()
