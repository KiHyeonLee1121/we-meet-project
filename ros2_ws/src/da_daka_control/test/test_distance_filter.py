"""Tests for the range filtering core."""

import math

from da_daka_control.distance_filter import DistanceFilterCore


def test_isolated_spike_is_rejected_by_median_stage():
    filter_core = DistanceFilterCore(5, 1)
    values = [1.0, 1.0, 1.0, 3.0, 1.0]
    outputs = [filter_core.update(value) for value in values]
    assert outputs[-2] == 1.0
    assert outputs[-1] == 1.0


def test_moving_average_smooths_valid_change():
    filter_core = DistanceFilterCore(1, 3)
    outputs = [filter_core.update(value) for value in (1.0, 1.3, 1.6)]
    assert math.isclose(outputs[-1], 1.3)
