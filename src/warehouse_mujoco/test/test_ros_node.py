"""Pure helpers of the ROS front end."""

from warehouse_mujoco.ros_node import _stamp


def test_stamp_never_goes_back_at_second_boundaries():
    """Accumulated 2 ms steps land just below whole seconds."""
    seconds, previous = 0.0, -1
    for _ in range(5000):
        seconds += 0.002
        stamp = _stamp(seconds)
        now = stamp.sec * 1000000000 + stamp.nanosec
        assert 0 <= stamp.nanosec < 1000000000
        assert now > previous
        previous = now
    assert (stamp.sec, stamp.nanosec) == (10, 0)
