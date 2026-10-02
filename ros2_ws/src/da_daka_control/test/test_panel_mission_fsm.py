"""Tests for the independent panel-movement mission state."""

import math

from da_daka_control.panel_mission_fsm import (
    advance_position_setpoint,
    control_ownership_failures,
    PanelMissionFsm,
    PanelMissionState,
    PanelRoute,
    RelativeWaypoint,
    StableArrival,
)
import pytest


def make_route() -> PanelRoute:
    """Create a small deterministic two-panel route."""
    return PanelRoute(
        [
            RelativeWaypoint(1.0, 0.0),
            RelativeWaypoint(1.0, 1.0),
        ]
    )


def test_route_rejects_empty_and_nonfinite_coordinates():
    with pytest.raises(ValueError):
        PanelRoute([])
    with pytest.raises(ValueError):
        RelativeWaypoint(math.nan, 0.0)


def test_start_resets_route_and_enters_precheck():
    mission = PanelMissionFsm(make_route())
    mission.start()
    assert mission.state == PanelMissionState.PRECHECK
    assert mission.active
    with pytest.raises(RuntimeError):
        mission.start()


def test_panel_hold_advances_then_finishes_in_loiter():
    mission = PanelMissionFsm(make_route())
    mission.start()
    mission.transition(PanelMissionState.MOVE_TO_PANEL)
    mission.panel_reached()
    mission.panel_hold_complete()
    assert mission.state == PanelMissionState.MOVE_TO_PANEL
    assert mission.route.index == 1
    mission.panel_reached()
    mission.panel_hold_complete()
    assert mission.state == PanelMissionState.FINAL_LOITER


def test_abort_latches_reason():
    mission = PanelMissionFsm(make_route())
    mission.start()
    mission.abort('QGC mode override')
    assert mission.state == PanelMissionState.ABORT
    assert mission.reason == 'QGC mode override'
    assert not mission.active


def test_arrival_requires_continuous_low_speed_hold():
    arrival = StableArrival(0.2, 0.1, 2.0)
    assert not arrival.update(
        position_error_m=0.1,
        speed_mps=0.05,
        now_s=10.0,
    )
    assert not arrival.update(
        position_error_m=0.1,
        speed_mps=0.2,
        now_s=11.0,
    )
    assert not arrival.update(
        position_error_m=0.1,
        speed_mps=0.05,
        now_s=12.0,
    )
    assert arrival.update(
        position_error_m=0.1,
        speed_mps=0.05,
        now_s=14.0,
    )


def test_arrival_resets_when_telemetry_is_stale():
    arrival = StableArrival(0.2, 0.1, 2.0)
    assert not arrival.update(
        position_error_m=0.1,
        speed_mps=0.05,
        now_s=10.0,
    )
    assert not arrival.update(
        position_error_m=0.0,
        speed_mps=0.0,
        now_s=11.0,
        telemetry_valid=False,
    )
    assert not arrival.update(
        position_error_m=0.1,
        speed_mps=0.05,
        now_s=12.0,
    )


def test_control_ownership_accepts_one_inactive_owner_per_topic():
    assert control_ownership_failures(
        distance_control_enabled=False,
        vertical_control_mode='DISABLED',
        distance_mission_publishers=0,
        distance_mission_state='',
        velocity_setpoint_publishers=1,
        position_setpoint_publishers=1,
    ) == []


@pytest.mark.parametrize(
    'overrides, expected',
    [
        ({'distance_control_enabled': True}, 'LiDAR distance control'),
        ({'vertical_control_mode': 'LOCAL_TAKEOFF'}, 'vertical controller'),
        (
            {
                'distance_mission_publishers': 1,
                'distance_mission_state': 'ARMING',
            },
            'distance mission',
        ),
        ({'velocity_setpoint_publishers': 2}, 'vertical setpoint'),
        ({'position_setpoint_publishers': 2}, 'position setpoint'),
    ],
)
def test_control_ownership_rejects_conflicts(overrides, expected):
    values = {
        'distance_control_enabled': False,
        'vertical_control_mode': 'DISABLED',
        'distance_mission_publishers': 0,
        'distance_mission_state': '',
        'velocity_setpoint_publishers': 1,
        'position_setpoint_publishers': 1,
    }
    values.update(overrides)
    failures = control_ownership_failures(**values)
    assert any(expected in failure for failure in failures)


def test_position_setpoint_advances_at_configured_horizontal_speed():
    result = advance_position_setpoint(
        (0.0, 0.0, 1.0),
        (0.0, -3.0, 1.0),
        maximum_horizontal_speed_mps=0.3,
        maximum_vertical_speed_mps=0.2,
        dt_s=1.0,
    )
    assert result == pytest.approx((0.0, -0.3, 1.0))


def test_position_setpoint_does_not_overshoot_target():
    result = advance_position_setpoint(
        (0.0, -2.9, 1.0),
        (0.0, -3.0, 1.0),
        maximum_horizontal_speed_mps=0.3,
        maximum_vertical_speed_mps=0.2,
        dt_s=1.0,
    )
    assert result == pytest.approx((0.0, -3.0, 1.0))


def test_position_setpoint_limits_vertical_change_independently():
    result = advance_position_setpoint(
        (0.0, 0.0, 1.0),
        (3.0, 0.0, 2.0),
        maximum_horizontal_speed_mps=0.3,
        maximum_vertical_speed_mps=0.2,
        dt_s=1.0,
    )
    assert result == pytest.approx((0.3, 0.0, 1.2))
