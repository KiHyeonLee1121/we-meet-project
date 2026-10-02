"""Tests for distance PID sign, deadband and output limits."""

import math

from da_daka_control.distance_controller import (
    DistancePid,
    LidarTakeoffController,
    LocalTakeoffController,
    TargetStabilityDetector,
    WindowedDistanceRate,
    yaw_rate_command,
)
import pytest


def test_yaw_rate_command_corrects_shortest_direction_and_limits_rate():
    rate = yaw_rate_command(
        target_rad=math.radians(-179.0),
        current_rad=math.radians(179.0),
        kp=1.0,
        maximum_rate_rad_s=0.35,
    )
    assert rate == pytest.approx(math.radians(2.0))

    limited = yaw_rate_command(
        target_rad=math.radians(40.0),
        current_rad=0.0,
        kp=1.0,
        maximum_rate_rad_s=0.35,
    )
    assert limited == pytest.approx(0.35)


def make_controller() -> DistancePid:
    """Create a controller with deterministic P-only settings."""
    return DistancePid(
        target_distance_m=1.0,
        deadband_m=0.05,
        kp=0.6,
        ki=0.0,
        kd=0.0,
        integral_limit=0.3,
        max_speed_mps=0.25,
        slow_zone_m=0.30,
        max_accel_mps2=10.0,
    )


def test_above_target_commands_downward_velocity():
    speed, error = make_controller().update(1.5, 0.05)
    assert error < 0.0
    assert math.isclose(speed, -0.25)


def test_below_target_commands_upward_velocity():
    speed, error = make_controller().update(0.8, 0.05)
    assert error > 0.0
    assert speed > 0.0


def test_deadband_commands_stop():
    controller = make_controller()
    controller.update(1.5, 0.05)
    speed, _ = controller.update(1.03, 0.05)
    assert speed == 0.0


def make_derivative_controller() -> DistancePid:
    """Create a controller that exposes only the filtered derivative term."""
    return DistancePid(
        target_distance_m=1.0,
        deadband_m=0.0,
        kp=0.0,
        ki=0.0,
        kd=1.0,
        integral_limit=0.0,
        max_speed_mps=1.0,
        slow_zone_m=1.0,
        max_accel_mps2=10.0,
    )


def test_pid_derivative_uses_supplied_windowed_distance_rate():
    controller = make_derivative_controller()
    speed, _ = controller.update(
        1.2,
        0.05,
        measured_distance_rate_mps=-0.2,
    )
    assert math.isclose(speed, 0.2)


def test_pid_omits_derivative_until_windowed_rate_is_available():
    controller = make_derivative_controller()
    controller.update(1.5, 0.05)
    speed, _ = controller.update(1.2, 0.05)
    assert speed == 0.0


def make_takeoff_controller() -> LocalTakeoffController:
    """Create a deterministic launch-relative Local Z controller."""
    return LocalTakeoffController(
        climb_height_m=1.1,
        tolerance_m=0.05,
        kp=0.8,
        max_speed_mps=0.5,
        slow_zone_m=0.4,
        max_accel_mps2=1.0,
    )


def test_local_takeoff_uses_launch_reference_not_global_zero():
    controller = make_takeoff_controller()
    assert math.isclose(controller.latch_launch_z(-32.4), -31.3)
    speed, error = controller.update(-32.4, 1.0)
    assert error > 0.0
    assert math.isclose(speed, 0.5)


def test_local_takeoff_duplicate_latch_keeps_original_target():
    controller = make_takeoff_controller()
    original_target = controller.latch_launch_z(10.0)

    duplicate_target = controller.latch_launch_z(10.7)

    assert math.isclose(original_target, 11.1)
    assert math.isclose(duplicate_target, original_target)
    assert math.isclose(controller.launch_z_m, 10.0)
    assert math.isclose(controller.target_z_m, 11.1)


def test_local_takeoff_respects_acceleration_and_speed_limits():
    controller = make_takeoff_controller()
    controller.latch_launch_z(4.0)
    first_speed, _ = controller.update(4.0, 0.1)
    second_speed, _ = controller.update(4.0, 0.1)
    assert math.isclose(first_speed, 0.1)
    assert math.isclose(second_speed, 0.2)
    for _ in range(10):
        speed, _ = controller.update(4.0, 0.1)
    assert math.isclose(speed, 0.5)


def test_local_takeoff_stops_inside_target_tolerance():
    controller = make_takeoff_controller()
    target_z = controller.latch_launch_z(10.0)
    controller.update(10.0, 1.0)
    speed, error = controller.update(target_z - 0.02, 0.1)
    assert abs(error) <= 0.05
    assert speed == 0.0


def test_local_takeoff_corrects_overshoot_downward():
    controller = make_takeoff_controller()
    target_z = controller.latch_launch_z(-5.0)
    speed, error = controller.update(target_z + 0.2, 1.0)
    assert error < 0.0
    assert speed < 0.0


def test_local_takeoff_rejects_missing_or_invalid_reference():
    controller = make_takeoff_controller()
    try:
        controller.update(0.0, 0.1)
    except RuntimeError:
        pass
    else:
        raise AssertionError('update before launch latch must fail')

    try:
        controller.latch_launch_z(math.nan)
    except ValueError:
        pass
    else:
        raise AssertionError('non-finite launch Z must fail')


def make_lidar_takeoff_controller() -> LidarTakeoffController:
    """Create a deterministic absolute LiDAR-distance takeoff controller."""
    return LidarTakeoffController(
        target_distance_m=1.1,
        tolerance_m=0.05,
        kp=0.8,
        max_speed_mps=0.5,
        slow_zone_m=0.4,
        max_accel_mps2=1.0,
    )


def test_lidar_takeoff_uses_absolute_distance_without_ground_offset():
    controller = make_lidar_takeoff_controller()
    speed, error = controller.update(0.28, 1.0)
    assert math.isclose(error, 0.82)
    assert math.isclose(speed, 0.5)


def test_lidar_takeoff_stops_at_distance_target():
    controller = make_lidar_takeoff_controller()
    controller.update(0.28, 1.0)
    speed, error = controller.update(1.08, 0.1)
    assert abs(error) <= 0.05
    assert speed == 0.0


def test_lidar_takeoff_corrects_overshoot_downward():
    controller = make_lidar_takeoff_controller()
    speed, error = controller.update(1.3, 1.0)
    assert error < 0.0
    assert speed < 0.0


def test_lidar_takeoff_soft_launch_then_cruise_and_terminal_slowdown():
    controller = LidarTakeoffController(
        target_distance_m=3.0,
        tolerance_m=0.30,
        kp=0.4,
        max_speed_mps=0.40,
        slow_zone_m=1.0,
        max_accel_mps2=1.0,
        soft_launch_max_speed_mps=0.25,
        soft_launch_until_distance_m=0.80,
    )

    soft_speed, _ = controller.update(0.30, 1.0)
    assert math.isclose(soft_speed, 0.25)

    cruise_speed, _ = controller.update(1.00, 1.0)
    assert math.isclose(cruise_speed, 0.40)

    terminal_speed, _ = controller.update(2.50, 1.0)
    assert math.isclose(terminal_speed, 0.20)


def test_lidar_takeoff_rejects_incomplete_soft_launch_profile():
    with pytest.raises(ValueError, match='must both be zero or positive'):
        LidarTakeoffController(
            target_distance_m=3.0,
            tolerance_m=0.30,
            kp=0.4,
            max_speed_mps=0.40,
            slow_zone_m=1.0,
            max_accel_mps2=0.25,
            soft_launch_max_speed_mps=0.25,
        )


def test_lidar_rate_uses_half_second_window():
    estimator = WindowedDistanceRate(0.5)
    assert estimator.update(1.00, 10.00) is None
    assert estimator.update(1.01, 10.25) is None
    rate = estimator.update(1.01, 10.50)
    assert math.isclose(rate, 0.02)


def test_lidar_rate_ignores_single_quantized_sample_jump():
    estimator = WindowedDistanceRate(0.5)
    assert estimator.update(1.00, 20.00) is None
    assert estimator.update(1.01, 20.01) is None
    rate = estimator.update(1.01, 20.50)
    assert math.isclose(rate, 0.02)


def test_lidar_rate_resets_after_sensor_gap():
    estimator = WindowedDistanceRate(0.5)
    assert estimator.update(1.00, 30.00) is None
    assert estimator.update(1.00, 30.50) == 0.0
    assert estimator.update(1.00, 31.10) is None


def make_stability_detector() -> TargetStabilityDetector:
    """Create a detector requiring five continuous stable seconds."""
    return TargetStabilityDetector(
        tolerance_m=0.05,
        duration_s=5.0,
        max_speed_mps=0.05,
    )


def test_target_requires_continuous_stable_duration():
    detector = make_stability_detector()
    assert not detector.update(0.02, 0.0, 10.0)
    assert not detector.update(0.01, 0.0, 14.9)
    assert detector.update(0.01, 0.0, 15.0)


def test_target_timer_resets_outside_tolerance():
    detector = make_stability_detector()
    assert not detector.update(0.01, 0.0, 10.0)
    assert not detector.update(0.06, 0.0, 12.0)
    assert not detector.update(0.01, 0.0, 14.0)
    assert detector.update(0.01, 0.0, 19.0)


def test_target_rejects_motion_and_invalid_flight_state():
    detector = make_stability_detector()
    assert not detector.update(0.01, 0.06, 10.0)
    assert not detector.update(0.01, 0.0, 14.0, flight_state_valid=False)
    assert not detector.update(0.01, 0.0, 18.0)
    assert detector.update(0.01, 0.0, 23.0)


def test_target_rejects_actual_vertical_motion_inside_distance_tolerance():
    detector = make_stability_detector()
    assert not detector.update(0.01, 0.06, 10.0, telemetry_valid=True)
    assert not detector.update(0.01, 0.06, 15.0, telemetry_valid=True)


def test_target_accepts_stable_distance_and_actual_velocity_for_five_seconds():
    detector = make_stability_detector()
    assert not detector.update(0.01, 0.02, 10.0, telemetry_valid=True)
    assert not detector.update(0.01, 0.02, 14.9, telemetry_valid=True)
    assert detector.update(0.01, 0.02, 15.0, telemetry_valid=True)


def test_target_window_accepts_one_noisy_sample_at_ninety_percent():
    detector = TargetStabilityDetector(
        tolerance_m=0.10,
        duration_s=3.0,
        max_speed_mps=0.05,
        required_ratio=0.90,
    )
    reached = False
    for index in range(11):
        speed = 0.06 if index == 5 else 0.02
        reached = detector.update(0.02, speed, index * 0.3)
    assert reached


def test_target_window_rejects_more_than_ten_percent_noisy_samples():
    detector = TargetStabilityDetector(
        tolerance_m=0.10,
        duration_s=3.0,
        max_speed_mps=0.05,
        required_ratio=0.90,
    )
    reached = False
    for index in range(11):
        speed = 0.06 if index in {4, 5} else 0.02
        reached = detector.update(0.02, speed, index * 0.3)
    assert not reached


def test_velocity_telemetry_timeout_resets_stability_timer():
    detector = make_stability_detector()
    assert not detector.update(0.01, 0.02, 10.0, telemetry_valid=True)
    assert not detector.update(0.01, 0.0, 14.0, telemetry_valid=False)
    assert not detector.update(0.01, 0.02, 15.0, telemetry_valid=True)
    assert detector.update(0.01, 0.02, 20.0, telemetry_valid=True)
