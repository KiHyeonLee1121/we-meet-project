"""Pendulum-inspired single-stream network/compute optimizer.

This module is deliberately isolated from PX4/ROS flight control. It consumes
measured/profiled resource points and returns a recommendation; applying the
recommendation to an encoder or detector is a separate, opt-in concern.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable

import yaml


@dataclass(frozen=True, slots=True)
class DemandPoint:
    name: str
    bitrate_mbps: float
    inference_ms: float
    accuracy: float
    detector_backend: str = "onnx"
    model_path: str | None = None
    input_width: int = 640
    input_height: int = 640
    encoder_profile: str | None = None

    def validate(self) -> None:
        if not self.name.strip():
            raise ValueError("optimization profile name cannot be empty")
        if not math.isfinite(self.bitrate_mbps) or self.bitrate_mbps <= 0.0:
            raise ValueError(f"{self.name}: bitrate_mbps must be positive and finite")
        if not math.isfinite(self.inference_ms) or self.inference_ms <= 0.0:
            raise ValueError(f"{self.name}: inference_ms must be positive and finite")
        if not math.isfinite(self.accuracy) or not 0.0 <= self.accuracy <= 1.0:
            raise ValueError(f"{self.name}: accuracy must be within [0, 1]")
        if self.detector_backend not in {"opencv", "onnx"}:
            raise ValueError(f"{self.name}: detector_backend must be opencv or onnx")
        if min(self.input_width, self.input_height) < 1:
            raise ValueError(f"{self.name}: detector input dimensions must be positive")


@dataclass(frozen=True, slots=True)
class OptimizerConfig:
    enabled: bool = False
    mode: str = "observe"
    min_accuracy: float = 0.80
    target_fps: float = 30.0
    bandwidth_mbps: float = 20.0
    network_weight: float = 0.5
    compute_weight: float = 0.5
    switch_margin: float = 0.05
    decision_interval_s: float = 1.0
    scene_change_threshold: float = 0.20
    profiles: tuple[DemandPoint, ...] = ()

    def validate(self) -> None:
        if self.mode not in {"observe", "apply"}:
            raise ValueError("optimization.mode must be observe or apply")
        if not 0.0 <= self.min_accuracy <= 1.0:
            raise ValueError("optimization.min_accuracy must be within [0, 1]")
        if self.target_fps <= 0.0 or self.bandwidth_mbps <= 0.0:
            raise ValueError("optimization target_fps and bandwidth_mbps must be positive")
        if self.network_weight < 0.0 or self.compute_weight < 0.0:
            raise ValueError("optimization weights cannot be negative")
        if self.network_weight + self.compute_weight <= 0.0:
            raise ValueError("at least one optimization weight must be positive")
        if not 0.0 <= self.switch_margin < 1.0:
            raise ValueError("optimization.switch_margin must be within [0, 1)")
        if self.decision_interval_s <= 0.0:
            raise ValueError("optimization.decision_interval_s must be positive")
        if not 0.0 <= self.scene_change_threshold <= 1.0:
            raise ValueError("optimization.scene_change_threshold must be within [0, 1]")
        if self.enabled and not self.profiles:
            raise ValueError("optimization.profiles cannot be empty when enabled")
        names: set[str] = set()
        for point in self.profiles:
            point.validate()
            if point.name in names:
                raise ValueError(f"duplicate optimization profile name: {point.name}")
            names.add(point.name)

    @property
    def compute_budget_ms(self) -> float:
        return 1000.0 / self.target_fps


@dataclass(frozen=True, slots=True)
class SchedulerDecision:
    point: DemandPoint
    feasible: bool
    changed: bool
    reason: str
    score: float


class JointScheduler:
    """Select one cost-efficient Pareto point for a single live video stream.

    Pendulum's max-cost-gradient allocator is mainly needed for multiple users.
    DA-DAKA currently has one stream, so selection over the profiled Pareto
    frontier is sufficient and avoids unnecessary control-plane complexity.
    """

    def __init__(self, config: OptimizerConfig) -> None:
        config.validate()
        self.config = config
        self._points = tuple(config.profiles)
        self._by_name = {point.name: point for point in self._points}

    def pareto_frontier(self) -> tuple[DemandPoint, ...]:
        candidates = [p for p in self._points if p.accuracy >= self.config.min_accuracy]
        frontier: list[DemandPoint] = []
        for point in candidates:
            dominated = any(
                other.name != point.name
                and other.bitrate_mbps <= point.bitrate_mbps
                and other.inference_ms <= point.inference_ms
                and (
                    other.bitrate_mbps < point.bitrate_mbps
                    or other.inference_ms < point.inference_ms
                )
                for other in candidates
            )
            if not dominated:
                frontier.append(point)
        return tuple(sorted(frontier, key=lambda p: (p.inference_ms, p.bitrate_mbps)))

    def _cost(self, point: DemandPoint, bandwidth_mbps: float, compute_ms: float) -> float:
        total_weight = self.config.network_weight + self.config.compute_weight
        nw = self.config.network_weight / total_weight
        cw = self.config.compute_weight / total_weight
        return (
            nw * point.bitrate_mbps / max(bandwidth_mbps, 1e-9)
            + cw * point.inference_ms / max(compute_ms, 1e-9)
        )

    @staticmethod
    def _overload(point: DemandPoint, bandwidth_mbps: float, compute_ms: float) -> float:
        network = max(point.bitrate_mbps / max(bandwidth_mbps, 1e-9) - 1.0, 0.0)
        compute = max(point.inference_ms / max(compute_ms, 1e-9) - 1.0, 0.0)
        return network + compute

    def select(
        self,
        *,
        bandwidth_mbps: float | None = None,
        compute_budget_ms: float | None = None,
        current_profile: str | None = None,
    ) -> SchedulerDecision:
        bandwidth = (
            self.config.bandwidth_mbps
            if bandwidth_mbps is None
            else bandwidth_mbps
        )
        compute_ms = (
            self.config.compute_budget_ms
            if compute_budget_ms is None
            else compute_budget_ms
        )
        if bandwidth <= 0.0 or compute_ms <= 0.0:
            raise ValueError("runtime resource budgets must be positive")

        frontier = self.pareto_frontier()
        if not frontier:
            best = min(
                self._points,
                key=lambda p: (
                    -p.accuracy,
                    self._overload(p, bandwidth, compute_ms),
                ),
            )
            return SchedulerDecision(
                point=best,
                feasible=False,
                changed=best.name != current_profile,
                reason="accuracy_requirement_unmet_best_effort",
                score=self._overload(best, bandwidth, compute_ms),
            )

        feasible = [
            p
            for p in frontier
            if p.bitrate_mbps <= bandwidth and p.inference_ms <= compute_ms
        ]
        if feasible:
            selected = min(
                feasible,
                key=lambda p: (self._cost(p, bandwidth, compute_ms), -p.accuracy),
            )
            selected_cost = self._cost(selected, bandwidth, compute_ms)

            current = self._by_name.get(current_profile or "")
            current_feasible = (
                current is not None
                and current.accuracy >= self.config.min_accuracy
                and current.bitrate_mbps <= bandwidth
                and current.inference_ms <= compute_ms
            )
            if current_feasible:
                current_cost = self._cost(current, bandwidth, compute_ms)
                improvement = (current_cost - selected_cost) / max(current_cost, 1e-9)
                if selected.name != current.name and improvement < self.config.switch_margin:
                    selected = current
                    selected_cost = current_cost
                    return SchedulerDecision(
                        point=selected,
                        feasible=True,
                        changed=False,
                        reason="hysteresis_keep_current",
                        score=selected_cost,
                    )

            return SchedulerDecision(
                point=selected,
                feasible=True,
                changed=selected.name != current_profile,
                reason="cost_optimal_feasible",
                score=selected_cost,
            )

        selected = min(
            frontier,
            key=lambda p: (
                self._overload(p, bandwidth, compute_ms),
                self._cost(p, bandwidth, compute_ms),
                -p.accuracy,
            ),
        )
        return SchedulerDecision(
            point=selected,
            feasible=False,
            changed=selected.name != current_profile,
            reason="simultaneous_bottleneck_best_effort",
            score=self._overload(selected, bandwidth, compute_ms),
        )


def load_optimizer_config(path: str | Path) -> OptimizerConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError("optimizer configuration root must be a YAML mapping")

    section: Any = raw.get("optimization", raw)
    if not isinstance(section, dict):
        raise ValueError("optimization must be a YAML mapping")
    profile_values = section.get("profiles", [])
    if not isinstance(profile_values, list):
        raise ValueError("optimization.profiles must be a list")

    profiles: list[DemandPoint] = []
    for item in profile_values:
        if not isinstance(item, dict):
            raise ValueError("each optimization profile must be a mapping")
        profiles.append(DemandPoint(**item))

    scalar = {key: value for key, value in section.items() if key != "profiles"}
    config = OptimizerConfig(**scalar, profiles=tuple(profiles))
    config.validate()
    return config


def profile_names(points: Iterable[DemandPoint]) -> tuple[str, ...]:
    return tuple(point.name for point in points)
