"""Deterministic dynamic route planning for a random panel layout."""

from dataclasses import dataclass
import math
from typing import Iterable

from da_daka_control.panel_mapping import PanelTarget


@dataclass(frozen=True)
class RoutePlan:
    """Ordered panel targets and complete travel length including home."""

    targets: tuple[PanelTarget, ...]
    total_distance_m: float

    @property
    def panel_ids(self) -> tuple[int, ...]:
        """Return the ordered stable panel identifiers."""
        return tuple(target.panel_id for target in self.targets)


def _distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def route_distance(
    start_xy: tuple[float, float],
    targets: Iterable[PanelTarget],
    home_xy: tuple[float, float],
) -> float:
    """Measure start -> every target -> home distance in metres."""
    if not all(math.isfinite(value) for value in (*start_xy, *home_xy)):
        raise ValueError('route endpoints must be finite')
    current = start_xy
    total = 0.0
    for target in targets:
        point = (target.east_m, target.north_m)
        total += _distance(current, point)
        current = point
    return total + _distance(current, home_xy)


def _nearest_neighbour(
    start_xy: tuple[float, float],
    targets: tuple[PanelTarget, ...],
) -> list[PanelTarget]:
    remaining = list(targets)
    ordered = []
    current = start_xy
    while remaining:
        selected = min(
            remaining,
            key=lambda target: (
                _distance(current, (target.east_m, target.north_m)),
                target.panel_id,
            ),
        )
        ordered.append(selected)
        remaining.remove(selected)
        current = (selected.east_m, selected.north_m)
    return ordered


def _two_opt(
    start_xy: tuple[float, float],
    ordered: list[PanelTarget],
    home_xy: tuple[float, float],
) -> list[PanelTarget]:
    if len(ordered) < 3:
        return ordered
    best = ordered
    best_distance = route_distance(start_xy, best, home_xy)
    improved = True
    while improved:
        improved = False
        for first in range(len(best) - 1):
            for last in range(first + 1, len(best)):
                candidate = (
                    best[:first]
                    + list(reversed(best[first:last + 1]))
                    + best[last + 1:]
                )
                candidate_distance = route_distance(
                    start_xy,
                    candidate,
                    home_xy,
                )
                if candidate_distance + 1e-9 < best_distance:
                    best = candidate
                    best_distance = candidate_distance
                    improved = True
        # The deterministic scan reaches a local optimum before returning.
    return best


def plan_panel_route(
    start_xy: tuple[float, float],
    targets: Iterable[PanelTarget],
    home_xy: tuple[float, float],
    *,
    improve_with_two_opt: bool = True,
) -> RoutePlan:
    """Plan a deterministic short route for any detected panel layout."""
    target_tuple = tuple(targets)
    if not target_tuple:
        raise ValueError('at least one panel target is required')
    ids = [target.panel_id for target in target_tuple]
    if len(ids) != len(set(ids)):
        raise ValueError('panel IDs must be unique')
    if not all(
        math.isfinite(value)
        for target in target_tuple
        for value in (target.east_m, target.north_m)
    ):
        raise ValueError('panel coordinates must be finite')

    ordered = _nearest_neighbour(start_xy, target_tuple)
    if improve_with_two_opt:
        ordered = _two_opt(start_xy, ordered, home_xy)
    return RoutePlan(
        targets=tuple(ordered),
        total_distance_m=route_distance(start_xy, ordered, home_xy),
    )
