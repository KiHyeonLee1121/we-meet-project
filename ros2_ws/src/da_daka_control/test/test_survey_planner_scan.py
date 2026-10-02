"""Tests for yaw-aligned and offset survey rectangles."""

import math

from da_daka_control.survey_planner import (
    rectangular_scan_waypoints,
    survey_timeout_elapsed_s,
)
import pytest


def _body_frame(point, launch, yaw_rad):
    """Return (forward, left) metres of one point relative to the launch pose."""
    dx = point[0] - launch[0]
    dy = point[1] - launch[1]
    forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    left = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return forward, left


def test_world_aligned_default_is_unchanged():
    got = rectangular_scan_waypoints(0.906, 2.093, 3.0, 2.0)
    assert got == (
        (-0.594, 1.093),
        (2.406, 1.093),
        (2.406, 3.093),
        (-0.594, 3.093),
        (0.906, 2.093),
    )


def test_world_aligned_rectangle_puts_half_its_legs_behind():
    """The 2026-08-23 flight wasted two legs behind the aircraft."""
    launch = (0.906, 2.093)
    yaw = math.radians(-118.3)
    corners = rectangular_scan_waypoints(*launch, 3.0, 2.0)[:4]
    forwards = [_body_frame(c, launch, yaw)[0] for c in corners]
    assert sum(1 for f in forwards if f < 0) == 2


def test_yaw_aligned_rectangle_is_symmetric_in_body_frame():
    launch = (0.906, 2.093)
    yaw = math.radians(-118.3)
    corners = rectangular_scan_waypoints(*launch, 3.0, 2.0, yaw_rad=yaw)[:4]
    body = [_body_frame(c, launch, yaw) for c in corners]
    forwards = sorted(round(f, 6) for f, _ in body)
    lefts = sorted(round(left, 6) for _, left in body)
    assert forwards == [-1.0, -1.0, 1.0, 1.0]
    assert lefts == [-1.5, -1.5, 1.5, 1.5]


def test_forward_offset_moves_every_corner_ahead():
    launch = (0.906, 2.093)
    yaw = math.radians(-118.3)
    corners = rectangular_scan_waypoints(
        *launch, 3.0, 2.0, yaw_rad=yaw, forward_offset_m=1.0
    )[:4]
    forwards = sorted(round(_body_frame(c, launch, yaw)[0], 6) for c in corners)
    assert forwards == [0.0, 0.0, 2.0, 2.0]


def test_forward_offset_large_enough_puts_all_legs_ahead():
    launch = (0.0, 0.0)
    yaw = math.radians(37.0)
    corners = rectangular_scan_waypoints(
        *launch, 3.0, 2.0, yaw_rad=yaw, forward_offset_m=1.2
    )[:4]
    assert all(_body_frame(c, launch, yaw)[0] > 0.0 for c in corners)


def test_lateral_offset_shifts_left_in_body_frame():
    launch = (0.0, 0.0)
    yaw = math.radians(-90.0)
    corners = rectangular_scan_waypoints(
        *launch, 3.0, 2.0, yaw_rad=yaw, lateral_offset_m=0.5
    )[:4]
    lefts = sorted(round(_body_frame(c, launch, yaw)[1], 6) for c in corners)
    assert lefts == [-1.0, -1.0, 2.0, 2.0]


def test_return_point_is_always_the_launch_point():
    launch = (1.5, -2.5)
    for kwargs in ({}, {'yaw_rad': 0.7, 'forward_offset_m': 2.0}):
        assert rectangular_scan_waypoints(*launch, 3.0, 2.0, **kwargs)[-1] == launch


def test_rectangle_size_is_preserved_under_rotation():
    corners = rectangular_scan_waypoints(
        4.0, -1.0, 3.0, 2.0, yaw_rad=math.radians(23.0)
    )[:4]
    side_a = math.dist(corners[0], corners[1])
    side_b = math.dist(corners[1], corners[2])
    assert side_a == pytest.approx(3.0)
    assert side_b == pytest.approx(2.0)


def test_non_finite_yaw_is_rejected():
    with pytest.raises(ValueError):
        rectangular_scan_waypoints(0.0, 0.0, 3.0, 2.0, yaw_rad=float('nan'))


def test_non_finite_offset_is_rejected():
    with pytest.raises(ValueError):
        rectangular_scan_waypoints(
            0.0, 0.0, 3.0, 2.0, forward_offset_m=float('inf')
        )


def test_non_positive_dimensions_are_rejected():
    with pytest.raises(ValueError):
        rectangular_scan_waypoints(0.0, 0.0, 0.0, 2.0)


def test_survey_timeout_prefers_scan_start():
    assert survey_timeout_elapsed_s(100.0, 10.0, 70.0) == pytest.approx(30.0)
    assert survey_timeout_elapsed_s(100.0, 10.0, None) == pytest.approx(90.0)
