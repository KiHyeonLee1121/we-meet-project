"""Unit tests for deterministic virtual distance motion."""

from da_daka_control.virtual_distance_sensor import move_toward


def test_move_toward_decreases_without_overshoot():
    assert move_toward(1.5, 1.0, 0.2) == 1.3
    assert move_toward(1.1, 1.0, 0.2) == 1.0


def test_move_toward_increases_without_overshoot():
    assert move_toward(0.5, 1.0, 0.2) == 0.7
    assert move_toward(0.9, 1.0, 0.2) == 1.0
