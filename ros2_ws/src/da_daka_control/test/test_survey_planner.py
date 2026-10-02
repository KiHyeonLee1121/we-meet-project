"""Tests for localization survey route generation and timeout accounting."""

from da_daka_control.survey_planner import (
    rectangular_scan_waypoints,
    survey_timeout_elapsed_s,
)
import pytest


def test_rectangular_scan_visits_corners_and_returns_to_launch():
    assert rectangular_scan_waypoints(10.0, -4.0, 3.0, 2.0) == (
        (8.5, -5.0),
        (11.5, -5.0),
        (11.5, -3.0),
        (8.5, -3.0),
        (10.0, -4.0),
    )


@pytest.mark.parametrize('width,depth', [(0.0, 2.0), (3.0, -1.0)])
def test_rectangular_scan_rejects_nonpositive_dimensions(width, depth):
    with pytest.raises(ValueError, match='positive'):
        rectangular_scan_waypoints(0.0, 0.0, width, depth)


def test_survey_timeout_restarts_when_active_scan_begins():
    assert survey_timeout_elapsed_s(120.0, 100.0, None) == 20.0
    assert survey_timeout_elapsed_s(120.0, 100.0, 115.0) == 5.0
