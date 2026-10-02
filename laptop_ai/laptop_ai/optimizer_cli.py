"""CLI to inspect a profiled network/compute demand curve safely."""

from __future__ import annotations

import argparse
import json

from laptop_ai.joint_optimizer import JointScheduler, load_optimizer_config


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate DA-DAKA joint optimization profiles")
    parser.add_argument("--config", required=True)
    parser.add_argument("--bandwidth-mbps", type=float, default=None)
    parser.add_argument("--compute-ms", type=float, default=None)
    parser.add_argument("--current-profile", default=None)
    args = parser.parse_args()

    config = load_optimizer_config(args.config)
    scheduler = JointScheduler(config)
    decision = scheduler.select(
        bandwidth_mbps=args.bandwidth_mbps,
        compute_budget_ms=args.compute_ms,
        current_profile=args.current_profile,
    )
    payload = {
        "profile": decision.point.name,
        "feasible": decision.feasible,
        "changed": decision.changed,
        "reason": decision.reason,
        "score": decision.score,
        "bitrate_mbps": decision.point.bitrate_mbps,
        "inference_ms": decision.point.inference_ms,
        "accuracy": decision.point.accuracy,
        "pareto_frontier": [point.name for point in scheduler.pareto_frontier()],
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
