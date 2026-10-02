"""Tests for the fail-closed solenoid bench interlocks."""

from da_daka_control.solenoid_bench_core import (
    bench_interlock_failures,
    BenchSnapshot,
    digicam_command_parameters,
    LANDED_STATE_ON_GROUND,
)


def ready_snapshot(**overrides) -> BenchSnapshot:
    """Return a snapshot that is safe for one ground pulse."""
    values = {
        'now_s': 10.0,
        'configuration_approved': True,
        'connected': True,
        'armed': False,
        'landed_state': LANDED_STATE_ON_GROUND,
        'state_time_s': 9.8,
        'extended_state_time_s': 9.8,
        'command_pending': False,
        'attempts': 0,
        'last_attempt_time_s': None,
    }
    values.update(overrides)
    return BenchSnapshot(**values)


def failures(snapshot: BenchSnapshot) -> list[str]:
    """Evaluate a snapshot with the package's default timing limits."""
    return bench_interlock_failures(
        snapshot,
        telemetry_timeout_s=1.0,
        cooldown_s=5.0,
        max_pulses_per_session=3,
    )


def test_ready_disarmed_ground_vehicle_passes() -> None:
    assert failures(ready_snapshot()) == []


def test_default_lock_blocks_output() -> None:
    blocked = failures(ready_snapshot(configuration_approved=False))
    assert 'bench_test_approved is false' in blocked


def test_armed_or_airborne_vehicle_is_blocked() -> None:
    armed = failures(ready_snapshot(armed=True))
    airborne = failures(ready_snapshot(landed_state=2))
    assert 'vehicle is not confirmed disarmed' in armed
    assert 'vehicle is not confirmed on ground' in airborne


def test_missing_or_stale_telemetry_is_blocked() -> None:
    missing = failures(
        ready_snapshot(state_time_s=None, extended_state_time_s=None)
    )
    stale = failures(
        ready_snapshot(state_time_s=8.9, extended_state_time_s=8.9)
    )
    assert 'MAVROS state is unavailable or stale' in missing
    assert 'MAVROS extended state is unavailable or stale' in missing
    assert 'MAVROS state is unavailable or stale' in stale
    assert 'MAVROS extended state is unavailable or stale' in stale


def test_pending_cooldown_and_attempt_limit_are_blocked() -> None:
    blocked = failures(
        ready_snapshot(
            command_pending=True,
            attempts=3,
            last_attempt_time_s=9.0,
        )
    )
    assert 'a trigger command is already pending' in blocked
    assert 'session pulse-attempt limit reached' in blocked
    assert 'cooldown active for 4.0s' in blocked


def test_command_is_one_shot_digicam_control() -> None:
    parameters = digicam_command_parameters()
    assert len(parameters) == 7
    assert parameters[4] == 1.0
    assert sum(parameters) == 1.0
