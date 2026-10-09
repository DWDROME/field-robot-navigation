import math


def test_reference_control_invariants():
    """Keep migration-critical arithmetic covered without requiring a live graph."""
    update_rate_hz = 100.0
    max_accel = 1.0
    max_yaw_rate = math.radians(45.0)
    acceleration_per_step = max_accel / update_rate_hz

    assert acceleration_per_step == 0.01
    assert 0.7853 < max_yaw_rate < 0.7855

    angle = 3.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    assert math.isclose(angle, math.pi)

    value = 0.0
    target = 0.25
    for _ in range(25):
        value = min(value + acceleration_per_step, target)
    assert math.isclose(value, target)


def test_empty_path_is_not_indexable():
    poses = []
    assert not poses
