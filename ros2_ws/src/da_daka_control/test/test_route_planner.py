"""Tests for random-layout panel route planning."""

from da_daka_control.panel_mapping import PanelTarget
from da_daka_control.route_planner import plan_panel_route, route_distance
import pytest


def panel(panel_id, east, north):
    return PanelTarget(panel_id, east, north, 1.0, 1.0, 0.9, 3)


def test_planner_uses_coordinates_not_input_order():
    targets = [panel(30, 3.0, 0.0), panel(10, 1.0, 0.0), panel(20, 2.0, 0.0)]
    plan = plan_panel_route((0.0, 0.0), targets, (0.0, 0.0))
    assert plan.panel_ids == (10, 20, 30)
    assert plan.total_distance_m == pytest.approx(6.0)


def test_two_opt_never_makes_nearest_neighbour_route_longer():
    targets = [
        panel(1, 1.0, 0.0),
        panel(2, 2.0, 2.0),
        panel(3, 0.0, 3.0),
        panel(4, 3.0, 0.5),
    ]
    nearest = plan_panel_route(
        (0.0, 0.0), targets, (0.0, 0.0), improve_with_two_opt=False
    )
    improved = plan_panel_route((0.0, 0.0), targets, (0.0, 0.0))
    assert improved.total_distance_m <= nearest.total_distance_m
    assert set(improved.panel_ids) == {1, 2, 3, 4}
    assert improved.total_distance_m == pytest.approx(
        route_distance((0.0, 0.0), improved.targets, (0.0, 0.0))
    )


def test_empty_survey_is_rejected():
    with pytest.raises(ValueError, match='at least one'):
        plan_panel_route((0.0, 0.0), [], (0.0, 0.0))
