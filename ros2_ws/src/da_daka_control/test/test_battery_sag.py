"""Regression tests for the 2026-08-23 false low-battery aborts."""

import math

from da_daka_control.mission_manager_node import (
    inflight_battery_failure,
    sag_compensated_remaining,
)

import pytest

PACK = {
    'cell_count': 4,
    'internal_resistance_ohm': 0.04,
    'empty_volts_per_cell': 3.50,
    'full_volts_per_cell': 4.05,
}


def test_hover_sag_does_not_read_as_a_flat_pack():
    """Flight 140930 logged 13.78 V at 22.85 A and PX4 called it 10%."""
    remaining = sag_compensated_remaining(
        voltage=13.78, current=-22.85, **PACK
    )
    assert remaining is not None
    # The same pack read 19-20% once it was back on the ground, so anything
    # near PX4's 10% would repeat the false abort.
    assert remaining > 0.20


def test_resting_reading_is_left_essentially_alone():
    """At 0.35 A there is no sag worth removing."""
    remaining = sag_compensated_remaining(
        voltage=14.81, current=-0.35, **PACK
    )
    assert remaining == pytest.approx(0.383, abs=0.01)


def test_compensation_tracks_the_same_pack_across_load():
    """Flight 140930 rest and hover samples must land close together."""
    rest = sag_compensated_remaining(voltage=14.81, current=-0.35, **PACK)
    hover = sag_compensated_remaining(voltage=13.78, current=-22.85, **PACK)
    # PX4 spread these two by 21 points (31% -> 10%).
    assert abs(rest - hover) < 0.08


def test_a_genuinely_empty_pack_still_reads_empty():
    remaining = sag_compensated_remaining(
        voltage=13.90, current=-0.20, **PACK
    )
    assert remaining < 0.10


def test_missing_current_falls_back_to_no_estimate():
    assert sag_compensated_remaining(voltage=14.8, current=None, **PACK) is None
    assert sag_compensated_remaining(voltage=None, current=-5.0, **PACK) is None
    assert sag_compensated_remaining(
        voltage=math.nan, current=-5.0, **PACK
    ) is None


def test_rejects_an_inverted_voltage_span():
    with pytest.raises(ValueError):
        sag_compensated_remaining(
            voltage=14.8,
            current=-5.0,
            cell_count=4,
            internal_resistance_ohm=0.04,
            empty_volts_per_cell=4.05,
            full_volts_per_cell=3.50,
        )


def test_rejects_a_nonpositive_cell_count():
    with pytest.raises(ValueError):
        sag_compensated_remaining(
            voltage=14.8,
            current=-5.0,
            cell_count=0,
            internal_resistance_ohm=0.04,
            empty_volts_per_cell=3.50,
            full_volts_per_cell=4.05,
        )


def test_a_transient_dip_does_not_abort_the_mission():
    reason = inflight_battery_failure(
        now_s=100.0,
        timeout_s=2.0,
        battery_remaining=0.08,
        battery_time_s=99.9,
        minimum_battery_remaining=0.10,
        below_since_s=99.0,
        sustained_below_s=3.0,
    )
    assert reason is None


def test_a_sustained_dip_still_aborts_the_mission():
    reason = inflight_battery_failure(
        now_s=100.0,
        timeout_s=2.0,
        battery_remaining=0.08,
        battery_time_s=99.9,
        minimum_battery_remaining=0.10,
        below_since_s=96.0,
        sustained_below_s=3.0,
    )
    assert reason == 'in-flight battery 8% below 10%'


def test_the_window_is_optional_for_existing_callers():
    reason = inflight_battery_failure(
        now_s=100.0,
        timeout_s=2.0,
        battery_remaining=0.08,
        battery_time_s=99.9,
        minimum_battery_remaining=0.10,
    )
    assert reason == 'in-flight battery 8% below 10%'


def test_stale_telemetry_never_aborts():
    assert inflight_battery_failure(
        now_s=100.0,
        timeout_s=2.0,
        battery_remaining=0.01,
        battery_time_s=90.0,
        minimum_battery_remaining=0.10,
        below_since_s=90.0,
        sustained_below_s=3.0,
    ) is None
