"""Tests for the forward-patrol and reverse-distance mission state."""

import math

from da_daka_control.panel_distance_mission_fsm import (
    advance_slowed_position_setpoint,
    body_offset_to_enu,
    early_takeoff_constant_position_allowed,
    horizontal_estimator_failures,
    lidar_referenced_local_z_target,
    MissionPhase,
    PanelDistanceMissionFsm,
    PanelDistanceMissionState,
    StableYawReference,
    TimeWindowMedian,
    wrapped_yaw_error,
)
from da_daka_control.panel_mission_fsm import PanelRoute, RelativeWaypoint
import pytest


def advance_horizontal_profile(
    current_xyz=(0.0, 0.0, 1.0),
    target_xyz=(3.0, 0.0, 1.0),
    actual_xy=(0.0, 0.0),
    current_speed=0.0,
    dt_s=0.1,
):
    """Advance the panel mission's configured horizontal profile once."""
    return advance_slowed_position_setpoint(
        current_xyz,
        target_xyz,
        actual_xy,
        current_horizontal_speed_mps=current_speed,
        maximum_horizontal_speed_mps=0.8,
        maximum_horizontal_accel_mps2=0.6,
        horizontal_slow_zone_m=1.2,
        minimum_approach_speed_mps=0.12,
        target_snap_distance_m=0.05,
        maximum_vertical_speed_mps=0.2,
        dt_s=dt_s,
    )


def test_horizontal_profile_accelerates_at_configured_limit():
    position, speed = advance_horizontal_profile()

    assert speed == pytest.approx(0.06)
    assert position == pytest.approx((0.006, 0.0, 1.0))


def test_horizontal_profile_cruises_at_point_eight_mps():
    position, speed = advance_horizontal_profile(current_speed=0.8)

    assert speed == pytest.approx(0.8)
    assert position == pytest.approx((0.08, 0.0, 1.0))


def test_horizontal_profile_slows_over_final_one_point_two_metres():
    position, speed = advance_horizontal_profile(
        current_xyz=(2.4, 0.0, 1.0),
        actual_xy=(2.4, 0.0),
        current_speed=0.4,
    )

    assert speed == pytest.approx(0.4)
    assert position == pytest.approx((2.44, 0.0, 1.0))


def test_actual_aircraft_position_can_trigger_early_slowdown():
    _, speed = advance_horizontal_profile(
        current_xyz=(2.0, 0.0, 1.0),
        actual_xy=(2.7, 0.0),
        current_speed=0.8,
    )

    assert speed == pytest.approx(0.74)


def test_horizontal_profile_keeps_minimum_speed_until_snap_zone():
    position, speed = advance_horizontal_profile(
        current_xyz=(2.9, 0.0, 1.0),
        actual_xy=(2.9, 0.0),
        current_speed=0.12,
    )

    assert speed == pytest.approx(0.12)
    assert position == pytest.approx((2.912, 0.0, 1.0))


def test_horizontal_profile_snaps_to_corner_inside_configured_zone():
    position, speed = advance_horizontal_profile(
        current_xyz=(2.96, 0.0, 0.8),
        actual_xy=(2.94, 0.0),
        current_speed=0.12,
    )

    assert speed == 0.0
    assert position == pytest.approx((3.0, 0.0, 0.82))


@pytest.mark.parametrize(
    ('yaw_deg', 'forward_m', 'left_m', 'east_m', 'north_m'),
    [
        (0.0, 3.0, 0.0, 3.0, 0.0),
        (90.0, 3.0, 0.0, 0.0, 3.0),
        (90.0, 0.0, 3.0, -3.0, 0.0),
        (-90.0, 3.0, 3.0, 3.0, -3.0),
    ],
)
def test_body_square_offsets_rotate_once_into_enu(
    yaw_deg,
    forward_m,
    left_m,
    east_m,
    north_m,
):
    actual_east, actual_north = body_offset_to_enu(
        forward_m=forward_m,
        left_m=left_m,
        launch_yaw_rad=math.radians(yaw_deg),
    )
    assert actual_east == pytest.approx(east_m)
    assert actual_north == pytest.approx(north_m)


def test_wrapped_yaw_error_uses_shortest_turn_across_pi():
    error = wrapped_yaw_error(math.radians(-179.0), math.radians(179.0))
    assert math.degrees(error) == pytest.approx(2.0)


def test_time_window_median_rejects_one_velocity_spike():
    detector = TimeWindowMedian(0.30)
    detector.update(0.02, 1.00)
    detector.update(0.04, 1.10)
    detector.update(0.80, 1.20)
    detector.update(0.03, 1.30)

    assert detector.value(1.30) == pytest.approx(0.035)


def test_time_window_median_requires_full_fresh_window():
    detector = TimeWindowMedian(0.30)
    detector.update(0.02, 1.00)
    detector.update(0.03, 1.20)
    assert detector.value(1.20) is None
    assert detector.value(1.60) is None

    detector.update(0.01, 2.00)
    assert detector.value(2.00) is None


def test_horizontal_estimator_accepts_relative_or_absolute_position():
    failures = horizontal_estimator_failures(
        now_s=10.0,
        timeout_s=0.5,
        estimator_time_s=9.9,
        attitude_valid=True,
        horizontal_velocity_valid=True,
        horizontal_relative_position_valid=False,
        horizontal_absolute_position_valid=True,
        constant_position_mode=False,
    )
    assert failures == []


def test_horizontal_estimator_rejects_constant_position_mode_in_flight():
    failures = horizontal_estimator_failures(
        now_s=10.0,
        timeout_s=0.5,
        estimator_time_s=9.9,
        attitude_valid=True,
        horizontal_velocity_valid=True,
        horizontal_relative_position_valid=True,
        horizontal_absolute_position_valid=True,
        constant_position_mode=True,
    )
    assert failures == ['PX4 estimator is in constant-position mode']


def test_takeoff_const_pos_grace_is_bounded_after_liftoff():
    assert early_takeoff_constant_position_allowed(
        landed_on_ground=False,
        offboard_takeoff_state=True,
        state_elapsed_s=1.9,
        airborne_grace_s=2.0,
    )
    assert not early_takeoff_constant_position_allowed(
        landed_on_ground=False,
        offboard_takeoff_state=True,
        state_elapsed_s=2.1,
        airborne_grace_s=2.0,
    )


def test_launch_yaw_latches_circular_mean_after_stable_full_window():
    detector = StableYawReference(
        duration_s=1.0,
        maximum_deviation_rad=math.radians(2.0),
    )
    assert detector.update(math.radians(179.5), 0.0) is None
    assert detector.update(math.radians(-179.5), 0.5) is None

    result = detector.update(math.radians(179.8), 1.0)

    assert result is not None
    assert abs(abs(math.degrees(result)) - 180.0) < 0.2


def test_launch_yaw_rejects_window_with_excessive_deviation():
    detector = StableYawReference(
        duration_s=1.0,
        maximum_deviation_rad=math.radians(2.0),
    )
    detector.update(math.radians(10.0), 0.0)
    detector.update(math.radians(15.0), 0.5)

    assert detector.update(math.radians(10.0), 1.0) is None


def test_launch_yaw_does_not_treat_a_long_sample_gap_as_stability():
    detector = StableYawReference(
        duration_s=1.0,
        maximum_deviation_rad=math.radians(2.0),
    )
    detector.update(math.radians(10.0), 0.0)

    assert detector.update(math.radians(10.0), 2.0) is None


def make_route(count: int = 4) -> PanelRoute:
    """Create a deterministic route with one waypoint per panel."""
    return PanelRoute(
        [RelativeWaypoint(float(index), 0.0) for index in range(count)]
    )


def arrive_and_finish_pause(mission: PanelDistanceMissionFsm) -> None:
    """Complete the phase-specific arrival sequence for one waypoint."""
    if mission.state == PanelDistanceMissionState.MOVE_TO_PANEL:
        mission.panel_move_arrived()
    if mission.phase is MissionPhase.DISTANCE:
        assert mission.state == PanelDistanceMissionState.ARRIVE_LOITER
        mission.transition(PanelDistanceMissionState.DISTANCE_LOITER)
        mission.distance_hold_complete()
    assert mission.state == PanelDistanceMissionState.PANEL_PAUSE
    mission.panel_pause_complete()


def test_patrols_one_to_four_then_distance_holds_four_to_one():
    mission = PanelDistanceMissionFsm(make_route())
    mission.start()
    mission.transition(PanelDistanceMissionState.MOVE_TO_PANEL)
    visited = []

    while mission.state != PanelDistanceMissionState.RETURN_HOME_PRESTREAM:
        visited.append((mission.phase, mission.route.index + 1))
        arrive_and_finish_pause(mission)
        if mission.state == PanelDistanceMissionState.MOVE_PRESTREAM:
            mission.transition(PanelDistanceMissionState.MOVE_TO_PANEL)

    assert visited == [
        (MissionPhase.PATROL, 1),
        (MissionPhase.PATROL, 2),
        (MissionPhase.PATROL, 3),
        (MissionPhase.PATROL, 4),
        (MissionPhase.DISTANCE, 4),
        (MissionPhase.DISTANCE, 3),
        (MissionPhase.DISTANCE, 2),
        (MissionPhase.DISTANCE, 1),
    ]


def test_final_reverse_panel_returns_home_before_landing():
    mission = PanelDistanceMissionFsm(make_route(1))
    mission.start()
    mission.phase = MissionPhase.DISTANCE
    mission.transition(PanelDistanceMissionState.DISTANCE_LOITER)

    mission.distance_hold_complete()
    mission.panel_pause_complete()

    assert mission.state == PanelDistanceMissionState.RETURN_HOME_PRESTREAM


def test_start_resets_route_and_phase_after_abort():
    mission = PanelDistanceMissionFsm(make_route(2))
    mission.start()
    mission.route.advance()
    mission.phase = MissionPhase.DISTANCE
    mission.abort('test abort')

    mission.start()

    assert mission.state == PanelDistanceMissionState.PRECHECK
    assert mission.phase is MissionPhase.PATROL
    assert mission.route.index == 0


def test_patrol_continues_offboard_and_starts_distance_at_panel_four():
    mission = PanelDistanceMissionFsm(make_route(2))
    mission.start()
    mission.transition(PanelDistanceMissionState.MOVE_TO_PANEL)

    mission.panel_move_arrived()
    mission.panel_pause_complete()
    assert mission.state == PanelDistanceMissionState.MOVE_TO_PANEL
    assert mission.route.index == 1

    mission.panel_move_arrived()
    mission.panel_pause_complete()
    assert mission.phase is MissionPhase.DISTANCE
    assert mission.state == PanelDistanceMissionState.ARRIVE_LOITER
    assert mission.route.index == 1


def test_reverse_pass_moves_directly_without_cruise_height_restore():
    mission = PanelDistanceMissionFsm(make_route(2))
    mission.start()
    mission.phase = MissionPhase.DISTANCE
    mission.route.advance()
    mission.transition(PanelDistanceMissionState.DISTANCE_LOITER)

    mission.distance_hold_complete()
    mission.panel_pause_complete()

    assert mission.route.index == 0
    assert mission.state == PanelDistanceMissionState.MOVE_PRESTREAM


def test_lidar_height_target_uses_local_z_only_as_relative_actuator():
    target_z = lidar_referenced_local_z_target(
        local_z_m=50.0,
        measured_distance_m=1.5,
        target_distance_m=2.0,
        gain=1.0,
        maximum_offset_m=0.4,
        tolerance_m=0.1,
    )
    assert target_z == pytest.approx(50.4)


def test_lidar_height_target_holds_current_z_inside_tolerance():
    target_z = lidar_referenced_local_z_target(
        local_z_m=-20.0,
        measured_distance_m=1.95,
        target_distance_m=2.0,
        gain=1.0,
        maximum_offset_m=0.4,
        tolerance_m=0.1,
    )
    assert target_z == pytest.approx(-20.0)


def test_lidar_height_target_limits_downward_correction():
    target_z = lidar_referenced_local_z_target(
        local_z_m=10.0,
        measured_distance_m=2.8,
        target_distance_m=2.0,
        gain=1.0,
        maximum_offset_m=0.3,
        tolerance_m=0.1,
    )
    assert target_z == pytest.approx(9.7)
