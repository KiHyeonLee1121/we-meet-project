"""Regression test for the 2026-08-23 ghost-cluster abort (flight 151600)."""

# The survey mapped three clusters from two physical panels: the real pair
# reached 24 and 23 observations while a ghost stopped at 3. The localization
# profile caps the map at two panels, so the extra cluster aborted a run that
# had otherwise found both panels.

from da_daka_control.panel_mapping import (
    MetricPanelObservation,
    PanelMapBuilder,
)

import pytest

REAL_A = (0.0, 0.0)
REAL_B = (2.0, 0.0)
GHOST = (4.0, 0.0)

FLIGHT_151600 = ((REAL_A, 24), (REAL_B, 23), (GHOST, 3))


def _build(minimum_observations):
    builder = PanelMapBuilder(
        merge_radius_m=0.45, minimum_observations=minimum_observations
    )
    for (east_m, north_m), count in FLIGHT_151600:
        for _ in range(count):
            builder.observe(
                MetricPanelObservation(
                    east_m=east_m,
                    north_m=north_m,
                    width_m=1.0,
                    height_m=1.7,
                    confidence=0.8,
                )
            )
    return builder


def test_the_flown_threshold_reproduces_the_abort():
    """minimum_observations=3 was in effect and yielded three panels."""
    assert len(_build(3).targets()) == 3


def test_the_new_threshold_drops_only_the_ghost():
    targets = _build(8).targets()
    assert len(targets) == 2
    assert sorted(target.observation_count for target in targets) == [23, 24]


def test_the_threshold_leaves_headroom_on_both_sides():
    """8 must sit clear of the ghost's 3 and the real panels' 23."""
    assert len(_build(4).targets()) == 2
    assert len(_build(23).targets()) == 2
    assert len(_build(24).targets()) == 1


def test_a_nonpositive_threshold_is_rejected():
    with pytest.raises(ValueError):
        PanelMapBuilder(merge_radius_m=0.45, minimum_observations=0)
