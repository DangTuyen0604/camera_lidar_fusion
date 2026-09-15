import math

import pytest


def rotation_drift_degrees(reference_yaw, candidate_yaw):
    delta = math.atan2(
        math.sin(candidate_yaw - reference_yaw),
        math.cos(candidate_yaw - reference_yaw),
    )
    return abs(math.degrees(delta))


@pytest.mark.parametrize(
    ('reference', 'candidate', 'expected'),
    [
        (0.0, 0.0, 0.0),
        (0.0, math.radians(2.0), 2.0),
        (math.radians(179.0), math.radians(-179.0), 2.0),
    ],
)
def test_rotation_drift_wraps_at_pi(reference, candidate, expected):
    assert rotation_drift_degrees(reference, candidate) == pytest.approx(
        expected
    )
