"""Unit tests for the independent launch-point altitude guard."""

from da_daka_control.altitude_guard import AltitudeGuardCore


def test_requires_positive_climb_limit():
    try:
        AltitudeGuardCore(0.0)
    except ValueError:
        pass
    else:
        raise AssertionError('zero climb limit must be rejected')


def test_arm_requires_ground_reference():
    guard = AltitudeGuardCore(5.0)
    assert guard.arm() is False
    assert guard.launch_z_m is None


def test_uses_launch_point_instead_of_global_zero():
    guard = AltitudeGuardCore(5.0)
    guard.observe_ground(12.4)
    assert guard.arm() is True
    assert abs(guard.update(16.9) - 4.5) < 1e-9
    assert guard.triggered is False


def test_latches_at_five_metres_above_launch():
    guard = AltitudeGuardCore(5.0)
    guard.observe_ground(-2.0)
    assert guard.arm() is True
    assert guard.update(3.0) == 5.0
    assert guard.triggered is True


def test_descent_does_not_trigger_guard():
    guard = AltitudeGuardCore(5.0)
    guard.observe_ground(3.0)
    assert guard.arm() is True
    assert guard.update(1.0) == -2.0
    assert guard.triggered is False


def test_disarm_clears_flight_reference_and_latch():
    guard = AltitudeGuardCore(5.0)
    guard.observe_ground(0.0)
    assert guard.arm() is True
    guard.update(5.1)
    assert guard.triggered is True
    guard.disarm()
    assert guard.launch_z_m is None
    assert guard.triggered is False
