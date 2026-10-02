from __future__ import annotations

import pytest

from laptop_ai.joint_optimizer import DemandPoint, JointScheduler, OptimizerConfig


def cfg(points, **kwargs):
    values = dict(
        enabled=True,
        min_accuracy=0.8,
        target_fps=30.0,
        bandwidth_mbps=10.0,
        network_weight=0.5,
        compute_weight=0.5,
        switch_margin=0.05,
        profiles=tuple(points),
    )
    values.update(kwargs)
    return OptimizerConfig(**values)


def test_pareto_frontier_removes_dominated_points() -> None:
    points = [
        DemandPoint("a", 2.0, 20.0, 0.85),
        DemandPoint("b", 5.0, 10.0, 0.86),
        DemandPoint("dominated", 6.0, 22.0, 0.90),
    ]
    scheduler = JointScheduler(cfg(points))
    assert [p.name for p in scheduler.pareto_frontier()] == ["b", "a"]


def test_network_bottleneck_prefers_lower_bitrate_when_feasible() -> None:
    points = [
        DemandPoint("low-net-heavy", 2.0, 20.0, 0.86),
        DemandPoint("high-net-light", 8.0, 6.0, 0.85),
    ]
    decision = JointScheduler(cfg(points)).select(bandwidth_mbps=3.0, compute_budget_ms=25.0)
    assert decision.feasible is True
    assert decision.point.name == "low-net-heavy"


def test_compute_bottleneck_prefers_lighter_model_when_network_allows() -> None:
    points = [
        DemandPoint("low-net-heavy", 2.0, 20.0, 0.86),
        DemandPoint("high-net-light", 8.0, 6.0, 0.85),
    ]
    decision = JointScheduler(cfg(points)).select(bandwidth_mbps=10.0, compute_budget_ms=8.0)
    assert decision.feasible is True
    assert decision.point.name == "high-net-light"


def test_simultaneous_bottleneck_is_explicit_best_effort() -> None:
    points = [
        DemandPoint("low-net-heavy", 2.0, 20.0, 0.86),
        DemandPoint("high-net-light", 8.0, 6.0, 0.85),
    ]
    decision = JointScheduler(cfg(points)).select(bandwidth_mbps=1.0, compute_budget_ms=5.0)
    assert decision.feasible is False
    assert decision.reason == "simultaneous_bottleneck_best_effort"


def test_hysteresis_avoids_small_switches() -> None:
    points = [
        DemandPoint("current", 4.0, 10.0, 0.85),
        DemandPoint("candidate", 3.9, 10.0, 0.85),
    ]
    decision = JointScheduler(cfg(points, switch_margin=0.10)).select(
        bandwidth_mbps=10.0,
        compute_budget_ms=30.0,
        current_profile="current",
    )
    assert decision.point.name == "current"
    assert decision.reason == "hysteresis_keep_current"


def test_invalid_runtime_budget_rejected() -> None:
    scheduler = JointScheduler(cfg([DemandPoint("a", 2.0, 10.0, 0.9)]))
    with pytest.raises(ValueError):
        scheduler.select(bandwidth_mbps=0.0)
