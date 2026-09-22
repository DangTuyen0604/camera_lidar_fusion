"""Validate measured Gate 12-14 artifacts, not source-code tokens."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def load(relative):
    """Load a committed measurement artifact."""
    return json.loads((ROOT / relative).read_text(encoding='utf-8'))


def test_canonical_demo_has_three_consecutive_complete_runs():
    document = load('benchmarks/canonical_demo_results.json')
    assert document['summary']['all_successful'] is True
    assert document['summary']['consecutive_successes'] >= 3
    assert len(document['runs']) >= 3
    for run in document['runs']:
        assert run['result'] == 'PASS'
        assert run['goal_status'] == 'SUCCEEDED'
        assert run['collision_count'] == 0
        assert run['camera_detection']['confidence'] > 0.0
        assert run['lidar_point_count'] > 0
        assert run['bridge_nonempty_clouds'] > 0
        assert run['evidence']['localized'] is True
        assert run['evidence']['static_map_occupied_cells'] > 0
        assert run['evidence']['nav_paths_received'] > 0
        assert run['evidence']['empty_bridge_heartbeats_after_removal'] >= 2


def test_required_live_metrics_are_positive():
    document = load('benchmarks/canonical_demo_results.json')
    for run in document['runs']:
        for field in (
                'camera_rate_hz', 'pointcloud_rate_hz', 'detection_rate_hz',
                'fusion_rate_hz', 'goal_duration_s',
                'obstacle_reaction_latency_ms'):
            assert run[field] > 0.0
        assert run['sync_delay_mean_ms'] >= 0.0


def test_core_faults_have_observable_seeded_effects():
    result = load('experiments/gate14_core_faults.json')
    assert result['seed'] == 12014
    timestamp = result['timestamp_delay']
    assert timestamp['injected_ms'] > timestamp['tolerance_ms']
    assert timestamp['healthy'] is False
    assert timestamp['drop_verified_by_core_test'] is True
    assert result['calibration_perturbation'][
        'projection_error_increase_px'] > 0.0
    assert result['pointcloud_noise']['measured_xyz_rms_change_m'] > 0.0
    density = result['density_reduction']
    assert density['output_points'] < density['input_points']
    assert density['deterministic_repeat_equal'] is True
    assert result['lost_detection']['obstacle_expired_in_every_run'] is True
