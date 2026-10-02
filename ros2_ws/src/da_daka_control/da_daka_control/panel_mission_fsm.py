"""Pure state machine for an independent panel-movement flight test."""

from dataclasses import dataclass
from enum import auto, Enum
import math
from typing import Optional, Sequence


INACTIVE_DISTANCE_MISSION_STATES = frozenset(
    {'IDLE', 'COMPLETE', 'ABORT'}
)


def control_ownership_failures(
    *,
    distance_control_enabled: bool,
    vertical_control_mode: str,
    distance_mission_publishers: int,
    distance_mission_state: str,
    velocity_setpoint_publishers: int,
    position_setpoint_publishers: int,
) -> list[str]:
    """Return conflicts that would create multiple flight-control owners."""
    failures = []
    if distance_control_enabled:
        failures.append('LiDAR distance control is enabled')
    if vertical_control_mode != 'DISABLED':
        failures.append(
            f'vertical controller is not disabled ({vertical_control_mode})'
        )
    if distance_mission_publishers > 0 and (
        distance_mission_state not in INACTIVE_DISTANCE_MISSION_STATES
    ):
        failures.append(
            f'distance mission is not inactive ({distance_mission_state})'
        )
    if velocity_setpoint_publishers != 1:
        failures.append(
            'expected exactly one vertical setpoint publisher '
            f'({velocity_setpoint_publishers} found)'
        )
    if position_setpoint_publishers != 1:
        failures.append(
            'expected exactly one position setpoint publisher '
            f'({position_setpoint_publishers} found)'
        )
    return failures


def advance_position_setpoint(
    current_xyz: tuple[float, float, float],
    target_xyz: tuple[float, float, float],
    *,
    maximum_horizontal_speed_mps: float,
    maximum_vertical_speed_mps: float,
    dt_s: float,
) -> tuple[float, float, float]:
    """Move a position setpoint toward its target with explicit limits."""
    if maximum_horizontal_speed_mps <= 0.0:
        raise ValueError('maximum_horizontal_speed_mps must be positive')
    if maximum_vertical_speed_mps <= 0.0:
        raise ValueError('maximum_vertical_speed_mps must be positive')
    if dt_s <= 0.0:
        raise ValueError('dt_s must be positive')
    if not all(math.isfinite(item) for item in current_xyz + target_xyz):
        raise ValueError('position setpoints must be finite')

    delta_x = target_xyz[0] - current_xyz[0]
    delta_y = target_xyz[1] - current_xyz[1]
    horizontal_distance = math.hypot(delta_x, delta_y)
    horizontal_step = maximum_horizontal_speed_mps * dt_s
    if horizontal_distance <= horizontal_step:
        next_x = target_xyz[0]
        next_y = target_xyz[1]
    else:
        scale = horizontal_step / horizontal_distance
        next_x = current_xyz[0] + delta_x * scale
        next_y = current_xyz[1] + delta_y * scale

    vertical_step = maximum_vertical_speed_mps * dt_s
    delta_z = target_xyz[2] - current_xyz[2]
    next_z = current_xyz[2] + max(
        -vertical_step,
        min(vertical_step, delta_z),
    )
    return (next_x, next_y, next_z)


class PanelMissionState(Enum):
    """States used by the independent panel-movement mission."""

    IDLE = auto()
    PRECHECK = auto()
    ARMING = auto()
    TAKEOFF = auto()
    TAKEOFF_PRESTREAM = auto()
    TAKEOFF_OFFBOARD = auto()
    TAKEOFF_HOLD = auto()
    LOITER_HANDOVER = auto()
    MOVE_PRESTREAM = auto()
    MOVE_OFFBOARD = auto()
    MOVE_TO_PANEL = auto()
    HOLD_PANEL = auto()
    FINAL_LOITER = auto()
    AUTO_LAND = auto()
    WAIT_DISARM = auto()
    COMPLETE = auto()
    ABORT = auto()


@dataclass(frozen=True)
class RelativeWaypoint:
    """Panel position in ENU meters relative to the launch point."""

    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.x_m) or not math.isfinite(self.y_m):
            raise ValueError('waypoint coordinates must be finite')


class StableArrival:
    """Require position and speed to remain settled continuously."""

    def __init__(
        self,
        position_tolerance_m: float,
        maximum_speed_mps: float,
        duration_s: float,
    ) -> None:
        if position_tolerance_m <= 0.0:
            raise ValueError('position_tolerance_m must be positive')
        if maximum_speed_mps < 0.0:
            raise ValueError('maximum_speed_mps cannot be negative')
        if duration_s <= 0.0:
            raise ValueError('duration_s must be positive')
        self.position_tolerance_m = position_tolerance_m
        self.maximum_speed_mps = maximum_speed_mps
        self.duration_s = duration_s
        self._stable_since_s: Optional[float] = None

    def reset(self) -> None:
        """Clear the continuous stability timer."""
        self._stable_since_s = None

    def update(
        self,
        *,
        position_error_m: float,
        speed_mps: float,
        now_s: float,
        telemetry_valid: bool = True,
    ) -> bool:
        """Return true after position and speed stay stable long enough."""
        stable = (
            telemetry_valid
            and math.isfinite(position_error_m)
            and math.isfinite(speed_mps)
            and position_error_m <= self.position_tolerance_m
            and speed_mps <= self.maximum_speed_mps
        )
        if not stable:
            self.reset()
            return False
        if self._stable_since_s is None:
            self._stable_since_s = now_s
        return now_s - self._stable_since_s >= self.duration_s


class PanelRoute:
    """Track a validated sequence of launch-relative panel waypoints."""

    def __init__(self, waypoints: Sequence[RelativeWaypoint]) -> None:
        if not waypoints:
            raise ValueError('at least one panel waypoint is required')
        self._waypoints = tuple(waypoints)
        self._index = 0

    @property
    def count(self) -> int:
        """Return the total number of panel waypoints."""
        return len(self._waypoints)

    @property
    def index(self) -> int:
        """Return the zero-based active waypoint index."""
        return self._index

    @property
    def current(self) -> RelativeWaypoint:
        """Return the active waypoint."""
        return self._waypoints[self._index]

    def reset(self) -> None:
        """Select the first panel."""
        self._index = 0

    def advance(self) -> bool:
        """Advance and return false when the final panel is complete."""
        if self._index + 1 >= len(self._waypoints):
            return False
        self._index += 1
        return True

    def retreat(self) -> bool:
        """Step backward and return false once the first panel is reached."""
        if self._index == 0:
            return False
        self._index -= 1
        return True


class PanelMissionFsm:
    """Hold mission state without performing ROS or MAVLink I/O."""

    ACTIVE_STATES = frozenset(
        state
        for state in PanelMissionState
        if state not in {
            PanelMissionState.IDLE,
            PanelMissionState.COMPLETE,
            PanelMissionState.ABORT,
        }
    )

    def __init__(self, route: PanelRoute) -> None:
        self.route = route
        self.state = PanelMissionState.IDLE
        self.reason = 'IDLE'

    @property
    def active(self) -> bool:
        """Return whether the mission owns an active flight sequence."""
        return self.state in self.ACTIVE_STATES

    def start(self) -> None:
        """Start a new mission from an inactive terminal state."""
        if self.active:
            raise RuntimeError(f'mission already active in {self.state.name}')
        self.route.reset()
        self.state = PanelMissionState.PRECHECK
        self.reason = 'mission requested'

    def transition(self, state: PanelMissionState, reason: str = '') -> None:
        """Move to a state and retain an operator-readable reason."""
        self.state = state
        self.reason = reason or state.name

    def panel_reached(self) -> None:
        """Enter the per-panel stable hold."""
        if self.state != PanelMissionState.MOVE_TO_PANEL:
            raise RuntimeError('panel_reached is only valid while moving')
        self.transition(PanelMissionState.HOLD_PANEL)

    def panel_hold_complete(self) -> None:
        """Advance to the next panel or begin the final LOITER handover."""
        if self.state != PanelMissionState.HOLD_PANEL:
            raise RuntimeError('panel hold is not active')
        if self.route.advance():
            self.transition(PanelMissionState.MOVE_TO_PANEL)
        else:
            self.transition(PanelMissionState.FINAL_LOITER)

    def abort(self, reason: str) -> None:
        """Latch an aborted terminal state."""
        self.transition(PanelMissionState.ABORT, reason)
