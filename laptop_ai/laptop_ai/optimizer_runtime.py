"""Non-invasive runtime glue for the joint optimizer.

The runtime defaults to observe-only. It never contacts PX4/MAVROS and never
changes Mission Manager state. Apply mode is intentionally limited to exposing
a selected profile to the laptop AI process; encoder control remains an external
adapter because this repository does not yet contain the Pi stream producer.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import time

from laptop_ai.joint_optimizer import JointScheduler, OptimizerConfig, SchedulerDecision


@dataclass(slots=True)
class RuntimeOptimizerState:
    current_profile: str | None = None
    last_decision_s: float = -float("inf")
    last_decision: SchedulerDecision | None = None


class RuntimeJointOptimizer:
    def __init__(self, config: OptimizerConfig) -> None:
        config.validate()
        self.config = config
        self.scheduler = JointScheduler(config)
        self.state = RuntimeOptimizerState()

    def maybe_decide(
        self,
        *,
        scene_changed: bool = False,
        bandwidth_mbps: float | None = None,
        now_s: float | None = None,
    ) -> SchedulerDecision | None:
        if not self.config.enabled:
            return None
        now = time.monotonic() if now_s is None else now_s
        if (
            not scene_changed
            and now - self.state.last_decision_s < self.config.decision_interval_s
        ):
            return None
        decision = self.scheduler.select(
            bandwidth_mbps=bandwidth_mbps,
            current_profile=self.state.current_profile,
        )
        self.state.last_decision_s = now
        self.state.last_decision = decision
        if self.config.mode == "apply" and decision.feasible:
            self.state.current_profile = decision.point.name
        return decision

    def log_decision(self, logger: logging.Logger, decision: SchedulerDecision) -> None:
        logger.info(
            "joint-opt mode=%s profile=%s feasible=%s changed=%s reason=%s "
            "bitrate=%.2fMbps infer=%.2fms accuracy=%.3f score=%.4f",
            self.config.mode,
            decision.point.name,
            decision.feasible,
            decision.changed,
            decision.reason,
            decision.point.bitrate_mbps,
            decision.point.inference_ms,
            decision.point.accuracy,
            decision.score,
        )
