"""Tests for spray-reaction physics, shaping and command bounds."""

import math

from da_daka_control.spray_reaction import (
    apply_vertical_feedforward,
    nozzle_area_m2,
    RampShaper,
    reaction_force_n,
    solve_operating_point,
)
import pytest


def supplied_operating_point():
    """Return the operating point for the supplied placeholder values."""
    return solve_operating_point(
        pump_open_flow_m3s=5.6 / 60000.0,
        pump_shutoff_pa=2.8e5,
        nozzle_area_m2_=nozzle_area_m2(0.006),
        discharge_coefficient=0.7,
    )


def test_nozzle_area_matches_known_six_millimetre_value():
    assert math.isclose(nozzle_area_m2(0.006), 2.8274e-5, rel_tol=1e-3)


def test_supplied_operating_point_matches_expected_range():
    point = supplied_operating_point()
    flow_lpm = point.flow_m3s * 60000.0
    assert 5.2 < flow_lpm < 5.5
    assert 0.05e5 < point.pressure_pa < 0.2e5


def test_reaction_force_matches_supplied_hand_calculation():
    point = supplied_operating_point()
    force_n = reaction_force_n(point.flow_m3s, point.velocity_mps)
    assert math.isclose(force_n, 0.286, rel_tol=0.05)


@pytest.mark.parametrize(
    'field',
    ('flow', 'pressure', 'area', 'coefficient', 'density'),
)
def test_operating_point_rejects_nonpositive_inputs(field):
    values = {
        'flow': 1.0,
        'pressure': 1.0,
        'area': 1.0,
        'coefficient': 1.0,
        'density': 1.0,
    }
    values[field] = 0.0
    with pytest.raises(ValueError):
        solve_operating_point(
            values['flow'],
            values['pressure'],
            values['area'],
            values['coefficient'],
            values['density'],
        )


def test_ramp_shaper_reaches_full_level_and_returns_to_zero():
    shaper = RampShaper(ramp_time_s=1.0)
    for _ in range(10):
        level = shaper.update(target_on=True, dt_s=0.1)
    assert math.isclose(level, 1.0, abs_tol=1e-6)
    assert math.isclose(
        shaper.update(target_on=False, dt_s=0.5),
        0.5,
        abs_tol=1e-6,
    )
    shaper.reset()
    assert shaper.level == 0.0


def test_zero_ramp_steps_immediately():
    shaper = RampShaper(ramp_time_s=0.0)
    assert shaper.update(True, 0.05) == 1.0
    assert shaper.update(False, 0.05) == 0.0


def test_feedforward_is_added_and_total_command_is_clamped():
    assert apply_vertical_feedforward(0.10, -0.04, 0.25) == pytest.approx(0.06)
    assert apply_vertical_feedforward(0.20, 0.20, 0.25) == pytest.approx(0.25)
    result = apply_vertical_feedforward(-0.20, -0.20, 0.25)
    assert result == pytest.approx(-0.25)


def test_feedforward_rejects_nonfinite_and_invalid_limit():
    with pytest.raises(ValueError):
        apply_vertical_feedforward(0.0, math.nan, 0.25)
    with pytest.raises(ValueError):
        apply_vertical_feedforward(0.0, 0.0, 0.0)
