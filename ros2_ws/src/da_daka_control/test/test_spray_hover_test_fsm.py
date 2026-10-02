"""Tests for the airborne spray-reaction test state machine."""

from da_daka_control.spray_hover_test_fsm import (
    horizontal_hold_speed,
    SprayHoverState,
    SprayHoverTestFsm,
    StableWindow,
    vertical_hold_speed,
)
import pytest


def _run_to(fsm: SprayHoverTestFsm, state: SprayHoverState) -> None:
    fsm.start()
    if state == SprayHoverState.PRECHECK:
        return
    fsm.precheck_complete()
    if state == SprayHoverState.ARMING:
        return
    fsm.armed()
    if state == SprayHoverState.TAKEOFF:
        return
    fsm.takeoff_complete()
    if state == SprayHoverState.HOVER_STABILISE:
        return
    fsm.hover_stable()
    if state == SprayHoverState.SPRAY_ARM:
        return
    fsm.spray_path_ready()
    if state == SprayHoverState.SPRAY:
        return
    fsm.spray_complete()
    if state == SprayHoverState.SETTLE:
        return
    fsm.settled()


def test_full_sequence_reaches_complete():
    fsm = SprayHoverTestFsm()
    _run_to(fsm, SprayHoverState.LAND)
    fsm.landing_started()
    fsm.disarmed()
    assert fsm.state == SprayHoverState.COMPLETE
    assert fsm.spray_fired is True
    assert fsm.active is False


def test_spray_is_not_reported_before_the_pulse_elapses():
    fsm = SprayHoverTestFsm()
    _run_to(fsm, SprayHoverState.SPRAY)
    assert fsm.spray_fired is False
    fsm.spray_complete()
    assert fsm.spray_fired is True


def test_spray_requires_a_settled_hover_first():
    fsm = SprayHoverTestFsm()
    _run_to(fsm, SprayHoverState.TAKEOFF)
    with pytest.raises(RuntimeError):
        fsm.spray_path_ready()


def test_landing_cannot_be_skipped_to_disarm():
    fsm = SprayHoverTestFsm()
    _run_to(fsm, SprayHoverState.LAND)
    with pytest.raises(RuntimeError):
        fsm.disarmed()


def test_abort_from_any_state_ends_the_run():
    fsm = SprayHoverTestFsm()
    _run_to(fsm, SprayHoverState.SPRAY)
    fsm.abort('valve stuck open')
    assert fsm.state == SprayHoverState.ABORT
    assert fsm.reason == 'valve stuck open'
    assert fsm.active is False
    assert fsm.airborne is False


def test_abort_requires_a_reason():
    fsm = SprayHoverTestFsm()
    fsm.start()
    with pytest.raises(ValueError):
        fsm.abort('')


def test_airborne_covers_every_flying_state():
    flying = {
        SprayHoverState.TAKEOFF,
        SprayHoverState.HOVER_STABILISE,
        SprayHoverState.SPRAY_ARM,
        SprayHoverState.SPRAY,
        SprayHoverState.SETTLE,
        SprayHoverState.LAND,
    }
    for state in flying:
        fsm = SprayHoverTestFsm()
        _run_to(fsm, state)
        assert fsm.airborne is True, state


def test_second_start_is_rejected_while_running():
    fsm = SprayHoverTestFsm()
    fsm.start()
    with pytest.raises(RuntimeError):
        fsm.start()


def test_stable_window_requires_continuous_holding():
    window = StableWindow(2.0)
    assert window.update(True, 0.0) is False
    assert window.update(True, 1.9) is False
    assert window.update(False, 1.95) is False
    assert window.update(True, 2.0) is False
    assert window.update(True, 3.9) is False
    assert window.update(True, 4.0) is True


def test_stable_window_reports_partial_elapsed_time():
    window = StableWindow(5.0)
    window.update(True, 10.0)
    assert window.elapsed_s(12.5) == pytest.approx(2.5)
    window.reset()
    assert window.elapsed_s(12.5) == 0.0


def test_stable_window_rejects_a_non_positive_duration():
    with pytest.raises(ValueError):
        StableWindow(0.0)


def test_vertical_hold_speed_is_bounded_both_ways():
    assert vertical_hold_speed(10.0, 0.8, 0.6) == pytest.approx(0.6)
    assert vertical_hold_speed(-10.0, 0.8, 0.6) == pytest.approx(-0.6)
    assert vertical_hold_speed(0.25, 0.8, 0.6) == pytest.approx(0.2)


def test_horizontal_hold_speed_is_bounded_both_ways():
    assert horizontal_hold_speed(4.0, 0.8, 0.5) == pytest.approx(0.5)
    assert horizontal_hold_speed(-4.0, 0.8, 0.5) == pytest.approx(-0.5)
    assert horizontal_hold_speed(0.5, 0.8, 0.5) == pytest.approx(0.4)


def test_hold_speeds_reject_invalid_bounds():
    with pytest.raises(ValueError):
        vertical_hold_speed(1.0, 0.8, 0.0)
    with pytest.raises(ValueError):
        horizontal_hold_speed(1.0, -0.1, 0.5)
