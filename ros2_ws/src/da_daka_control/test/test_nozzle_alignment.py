"""Tests for physical camera-to-nozzle image targeting."""

import math

from da_daka_control.nozzle_alignment import (
    body_velocity_to_enu,
    compute_image_velocity,
    nozzle_image_target,
    quaternion_yaw_rad,
)
from da_daka_control.panel_mapping import CameraGroundModel
import pytest


def test_zero_nozzle_offset_targets_image_center():
    target = nozzle_image_target(
        CameraGroundModel(1.0, 1.0),
        camera_to_nozzle_forward_m=0.0,
        camera_to_nozzle_left_m=0.0,
        distance_m=1.0,
    )
    assert target.x_norm == pytest.approx(0.5)
    assert target.y_norm == pytest.approx(0.5)
    assert target.inside_safe_frame


def test_forward_left_nozzle_offset_moves_image_target():
    target = nozzle_image_target(
        CameraGroundModel(2.0, 2.0),
        camera_to_nozzle_forward_m=0.2,
        camera_to_nozzle_left_m=0.1,
        distance_m=1.0,
    )
    assert target.x_norm == pytest.approx(0.45)
    assert target.y_norm == pytest.approx(0.40)


def test_visual_velocity_aligns_to_offset_target_not_frame_center():
    x_speed, y_speed, aligned = compute_image_velocity(
        observed_x_norm=0.45,
        observed_y_norm=0.40,
        target_x_norm=0.45,
        target_y_norm=0.40,
        deadband_norm=0.02,
        gain_mps_per_norm=0.4,
        maximum_speed_mps=0.1,
    )
    assert (x_speed, y_speed) == (0.0, 0.0)
    assert aligned


def test_measured_camera_to_nozzle_target_uses_camera_surface_distance():
    target = nozzle_image_target(
        CameraGroundModel(1.30, 0.73),
        camera_to_nozzle_forward_m=-0.07,
        camera_to_nozzle_left_m=-0.05,
        distance_m=1.0 - 0.16,
    )
    assert target.x_norm == pytest.approx(0.5457875458)
    assert target.y_norm == pytest.approx(0.6141552511)
    assert target.inside_safe_frame


def test_image_right_commands_body_right():
    forward_mps, left_mps, aligned = compute_image_velocity(
        observed_x_norm=0.60,
        observed_y_norm=0.50,
        target_x_norm=0.50,
        target_y_norm=0.50,
        deadband_norm=0.01,
        gain_mps_per_norm=0.4,
        maximum_speed_mps=0.1,
    )
    assert forward_mps == pytest.approx(0.0)
    assert left_mps == pytest.approx(-0.04)
    assert not aligned


@pytest.mark.parametrize(
    ('yaw_deg', 'east_mps', 'north_mps'),
    [
        (0.0, 0.10, 0.05),
        (90.0, -0.05, 0.10),
        (180.0, -0.10, -0.05),
    ],
)
def test_body_visual_velocity_rotates_to_map_enu(
    yaw_deg,
    east_mps,
    north_mps,
):
    east, north = body_velocity_to_enu(
        0.10,
        0.05,
        math.radians(yaw_deg),
    )
    assert east == pytest.approx(east_mps, abs=1e-9)
    assert north == pytest.approx(north_mps, abs=1e-9)


def test_quaternion_yaw_recovers_vehicle_heading():
    half_yaw = math.radians(90.0) / 2.0
    yaw = quaternion_yaw_rad(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    assert yaw == pytest.approx(math.pi / 2.0)
