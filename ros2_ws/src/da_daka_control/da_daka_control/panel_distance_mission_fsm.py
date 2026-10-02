"""Pure state machine: patrol a route once, then re-walk it holding 1 m."""

from collections import deque
from enum import auto, Enum
import math
import statistics
from typing import Deque, Optional

from da_daka_control.panel_mission_fsm import (  # noqa: F401 (re-exported)
    control_ownership_failures,
    PanelRoute,
    RelativeWaypoint,
    StableArrival,
)


def advance_slowed_position_setpoint(
    current_xyz: tuple[float, float, float],
    target_xyz: tuple[float, float, float],
    actual_xy: tuple[float, float],
    *,
    current_horizontal_speed_mps: float,
    maximum_horizontal_speed_mps: float,
    maximum_horizontal_accel_mps2: float,
    horizontal_slow_zone_m: float,
    minimum_approach_speed_mps: float,
    target_snap_distance_m: float,
    maximum_vertical_speed_mps: float,
    dt_s: float,
) -> tuple[tuple[float, float, float], float]:
    """Advance a position target with acceleration and terminal slowdown."""
    values = (
        *current_xyz,
        *target_xyz,
        *actual_xy,
        current_horizontal_speed_mps,
        maximum_horizontal_speed_mps,
        maximum_horizontal_accel_mps2,
        horizontal_slow_zone_m,
        minimum_approach_speed_mps,
        target_snap_distance_m,
        maximum_vertical_speed_mps,
        dt_s,
    )
    if not all(math.isfinite(item) for item in values):
        raise ValueError('slowed position setpoint inputs must be finite')
    if current_horizontal_speed_mps < 0.0:
        raise ValueError('current_horizontal_speed_mps cannot be negative')
    positive = (
        maximum_horizontal_speed_mps,
        maximum_horizontal_accel_mps2,
        horizontal_slow_zone_m,
        minimum_approach_speed_mps,
        target_snap_distance_m,
        maximum_vertical_speed_mps,
        dt_s,
    )
    if any(item <= 0.0 for item in positive):
        raise ValueError('slowed setpoint limits must be positive')
    if minimum_approach_speed_mps > maximum_horizontal_speed_mps:
        raise ValueError('minimum approach speed cannot exceed maximum speed')
    if target_snap_distance_m >= horizontal_slow_zone_m:
        raise ValueError('target snap distance must be below slow zone')

    command_dx = target_xyz[0] - current_xyz[0]
    command_dy = target_xyz[1] - current_xyz[1]
    command_remaining_m = math.hypot(command_dx, command_dy)
    actual_remaining_m = math.hypot(
        target_xyz[0] - actual_xy[0],
        target_xyz[1] - actual_xy[1],
    )
    if command_remaining_m <= target_snap_distance_m:
        next_x = target_xyz[0]
        next_y = target_xyz[1]
        next_horizontal_speed_mps = 0.0
    else:
        # Slow down as soon as either the moving command or the aircraft
        # enters the terminal zone. The command-distance term prevents the
        # moving target from reaching the waypoint at cruise speed while the
        # aircraft still carries momentum behind it.
        profile_remaining_m = min(command_remaining_m, actual_remaining_m)
        desired_speed_mps = maximum_horizontal_speed_mps * min(
            1.0,
            profile_remaining_m / horizontal_slow_zone_m,
        )
        desired_speed_mps = max(
            minimum_approach_speed_mps,
            desired_speed_mps,
        )
        maximum_speed_change_mps = maximum_horizontal_accel_mps2 * dt_s
        next_horizontal_speed_mps = max(
            current_horizontal_speed_mps - maximum_speed_change_mps,
            min(
                current_horizontal_speed_mps + maximum_speed_change_mps,
                desired_speed_mps,
            ),
        )
        horizontal_step_m = min(
            command_remaining_m,
            next_horizontal_speed_mps * dt_s,
        )
        scale = horizontal_step_m / command_remaining_m
        next_x = current_xyz[0] + command_dx * scale
        next_y = current_xyz[1] + command_dy * scale

    vertical_step_m = maximum_vertical_speed_mps * dt_s
    vertical_error_m = target_xyz[2] - current_xyz[2]
    next_z = current_xyz[2] + max(
        -vertical_step_m,
        min(vertical_step_m, vertical_error_m),
    )
    return (next_x, next_y, next_z), next_horizontal_speed_mps


def body_offset_to_enu(
    *,
    forward_m: float,
    left_m: float,
    launch_yaw_rad: float,
) -> tuple[float, float]:
    """Rotate a launch-body FLU offset into the fixed local ENU frame."""
    values = (forward_m, left_m, launch_yaw_rad)
    if not all(math.isfinite(item) for item in values):
        raise ValueError('body-to-ENU inputs must be finite')
    cosine = math.cos(launch_yaw_rad)
    sine = math.sin(launch_yaw_rad)
    return (
        cosine * forward_m - sine * left_m,
        sine * forward_m + cosine * left_m,
    )


def wrapped_yaw_error(target_rad: float, current_rad: float) -> float:
    """Return the shortest signed yaw error in radians."""
    if not math.isfinite(target_rad) or not math.isfinite(current_rad):
        raise ValueError('yaw angles must be finite')
    return (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi


class TimeWindowMedian:
    """Median-filter finite samples over a recent monotonic-time window."""

    def __init__(self, duration_s: float) -> None:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError('median window duration must be positive')
        self.duration_s = duration_s
        self._samples: Deque[tuple[float, float]] = deque()
        self._started_s: Optional[float] = None

    def reset(self) -> None:
        """Discard all samples and window coverage."""
        self._samples.clear()
        self._started_s = None

    def update(self, value: float, now_s: float) -> None:
        """Add one finite sample at a nondecreasing timestamp."""
        if not math.isfinite(value) or not math.isfinite(now_s):
            raise ValueError('median samples must be finite')
        if self._samples and now_s < self._samples[-1][0]:
            raise ValueError('median sample time cannot move backwards')
        if (
            self._samples
            and now_s - self._samples[-1][0] > self.duration_s
        ):
            self.reset()
        if self._started_s is None:
            self._started_s = now_s
        self._samples.append((now_s, value))
        self._prune(now_s)

    def value(self, now_s: float) -> Optional[float]:
        """Return a median only after a complete, fresh window exists."""
        if not math.isfinite(now_s):
            raise ValueError('median query time must be finite')
        self._prune(now_s)
        if (
            self._started_s is None
            or now_s - self._started_s < self.duration_s
            or not self._samples
            or now_s - self._samples[-1][0] > self.duration_s
        ):
            return None
        return float(statistics.median(value for _, value in self._samples))

    def _prune(self, now_s: float) -> None:
        cutoff_s = now_s - self.duration_s
        while self._samples and self._samples[0][0] < cutoff_s:
            self._samples.popleft()
        if not self._samples:
            self._started_s = None


def horizontal_estimator_failures(
    *,
    now_s: float,
    timeout_s: float,
    estimator_time_s: Optional[float],
    attitude_valid: Optional[bool],
    horizontal_velocity_valid: Optional[bool],
    horizontal_relative_position_valid: Optional[bool],
    horizontal_absolute_position_valid: Optional[bool],
    constant_position_mode: Optional[bool],
    allow_constant_position_mode: bool = False,
) -> list[str]:
    """Return PX4 estimator failures that make Local-XY flight unsafe."""
    if timeout_s <= 0.0:
        raise ValueError('estimator timeout must be positive')
    if not math.isfinite(now_s) or not math.isfinite(timeout_s):
        raise ValueError('estimator time values must be finite')
    if estimator_time_s is None or now_s - estimator_time_s > timeout_s:
        return ['PX4 estimator status unavailable or stale']
    failures = []
    if attitude_valid is not True:
        failures.append('PX4 attitude estimate is invalid')
    if horizontal_velocity_valid is not True:
        failures.append('PX4 horizontal velocity estimate is invalid')
    if not (
        horizontal_relative_position_valid is True
        or horizontal_absolute_position_valid is True
    ):
        failures.append('PX4 horizontal position estimate is invalid')
    if constant_position_mode is True and not allow_constant_position_mode:
        failures.append('PX4 estimator is in constant-position mode')
    return failures


def early_takeoff_constant_position_allowed(
    *,
    landed_on_ground: bool,
    offboard_takeoff_state: bool,
    state_elapsed_s: float,
    airborne_grace_s: float,
) -> bool:
    """Allow only PX4's bounded constant-position transition at takeoff."""
    if not math.isfinite(state_elapsed_s) or not math.isfinite(
        airborne_grace_s
    ):
        raise ValueError('takeoff const-pos timing must be finite')
    if state_elapsed_s < 0.0 or airborne_grace_s <= 0.0:
        raise ValueError('takeoff const-pos timing is outside safe bounds')
    return landed_on_ground or (
        offboard_takeoff_state and state_elapsed_s <= airborne_grace_s
    )


class StableYawReference:
    """Estimate a circular-mean yaw after a stable, full-duration window."""

    def __init__(self, duration_s: float, maximum_deviation_rad: float) -> None:
        if duration_s <= 0.0:
            raise ValueError('duration_s must be positive')
        if not 0.0 < maximum_deviation_rad < math.pi:
            raise ValueError('maximum_deviation_rad must be in (0, pi)')
        self.duration_s = duration_s
        self.maximum_deviation_rad = maximum_deviation_rad
        self.reset()

    def reset(self) -> None:
        """Clear collected yaw samples and the stable result."""
        self._samples: Deque[tuple[float, float]] = deque()
        self.stable_yaw_rad: Optional[float] = None

    def update(self, yaw_rad: float, now_s: float) -> Optional[float]:
        """Return circular-mean yaw once the complete window is stable."""
        if not math.isfinite(yaw_rad) or not math.isfinite(now_s):
            raise ValueError('yaw and time must be finite')
        if self._samples and now_s <= self._samples[-1][0]:
            self.reset()
        if (
            self._samples
            and now_s - self._samples[-1][0] >= self.duration_s
        ):
            self.reset()
        self._samples.append((now_s, yaw_rad))
        while (
            len(self._samples) >= 2
            and now_s - self._samples[1][0] >= self.duration_s
        ):
            self._samples.popleft()
        coverage_s = now_s - self._samples[0][0]
        if coverage_s < self.duration_s:
            self.stable_yaw_rad = None
            return None
        mean_yaw_rad = math.atan2(
            sum(math.sin(yaw) for _, yaw in self._samples),
            sum(math.cos(yaw) for _, yaw in self._samples),
        )
        maximum_deviation = max(
            abs(wrapped_yaw_error(mean_yaw_rad, yaw))
            for _, yaw in self._samples
        )
        self.stable_yaw_rad = (
            mean_yaw_rad
            if maximum_deviation <= self.maximum_deviation_rad
            else None
        )
        return self.stable_yaw_rad


def lidar_referenced_local_z_target(
    *,
    local_z_m: float,
    measured_distance_m: float,
    target_distance_m: float,
    gain: float,
    maximum_offset_m: float,
    tolerance_m: float,
) -> float:
    """Convert a LiDAR height error into a bounded relative Local-Z target."""
    values = (
        local_z_m,
        measured_distance_m,
        target_distance_m,
        gain,
        maximum_offset_m,
        tolerance_m,
    )
    if not all(math.isfinite(item) for item in values):
        raise ValueError('LiDAR height target inputs must be finite')
    if target_distance_m <= 0.0:
        raise ValueError('target_distance_m must be positive')
    if gain <= 0.0 or maximum_offset_m <= 0.0 or tolerance_m <= 0.0:
        raise ValueError('gain, maximum_offset_m and tolerance_m must be positive')

    error_m = target_distance_m - measured_distance_m
    if abs(error_m) <= tolerance_m:
        return local_z_m
    correction_m = max(
        -maximum_offset_m,
        min(maximum_offset_m, gain * error_m),
    )
    return local_z_m + correction_m


class MissionPhase(Enum):
    """Which of the two route passes is currently running."""

    PATROL = auto()
    DISTANCE = auto()


class PanelDistanceMissionState(Enum):
    """States for the two-pass route + per-panel distance-hold mission."""

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
    ARRIVE_LOITER = auto()
    DISTANCE_PRESTREAM = auto()
    DISTANCE_OFFBOARD = auto()
    DISTANCE_CONTROL = auto()
    DISTANCE_HOLD = auto()
    DISTANCE_LOITER = auto()
    PANEL_PAUSE = auto()
    RETURN_HOME_PRESTREAM = auto()
    RETURN_HOME_OFFBOARD = auto()
    RETURN_HOME = auto()
    FINAL_LOITER = auto()
    AUTO_LAND = auto()
    WAIT_DISARM = auto()
    COMPLETE = auto()
    ABORT = auto()


class PanelDistanceMissionFsm:
    """Hold mission state without performing ROS or MAVLink I/O."""

    # Pass 1 (PATROL) walks the route forward (index 0 -> N-1), pausing
    # briefly at each waypoint without touching the distance controller.
    # Pass 2 (DISTANCE) walks the same route backward from wherever PATROL
    # ended (index N-1 -> 0), holding target_distance_m at each waypoint
    # before a longer pause. Landing happens at whichever waypoint DISTANCE
    # finishes on.

    ACTIVE_STATES = frozenset(
        state
        for state in PanelDistanceMissionState
        if state not in {
            PanelDistanceMissionState.IDLE,
            PanelDistanceMissionState.COMPLETE,
            PanelDistanceMissionState.ABORT,
        }
    )

    def __init__(self, route: PanelRoute) -> None:
        self.route = route
        self.state = PanelDistanceMissionState.IDLE
        self.phase = MissionPhase.PATROL
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
        self.phase = MissionPhase.PATROL
        self.state = PanelDistanceMissionState.PRECHECK
        self.reason = 'mission requested'

    def transition(
        self,
        state: PanelDistanceMissionState,
        reason: str = '',
    ) -> None:
        """Move to a state and retain an operator-readable reason."""
        self.state = state
        self.reason = reason or state.name

    def panel_move_arrived(self) -> None:
        """Handle arrival at a waypoint: pause-only in PATROL, hold in DISTANCE."""
        if self.state != PanelDistanceMissionState.MOVE_TO_PANEL:
            raise RuntimeError('panel_move_arrived is only valid while moving')
        if self.phase is MissionPhase.PATROL:
            self.transition(PanelDistanceMissionState.PANEL_PAUSE)
        else:
            self.transition(PanelDistanceMissionState.ARRIVE_LOITER)

    def distance_hold_complete(self) -> None:
        """Enter the post-hold pause once the distance controller is off."""
        if self.state != PanelDistanceMissionState.DISTANCE_LOITER:
            raise RuntimeError('distance loiter is not active')
        self.transition(PanelDistanceMissionState.PANEL_PAUSE)

    def panel_pause_complete(self) -> None:
        """Advance the route (direction depends on phase) or finish up."""
        if self.state != PanelDistanceMissionState.PANEL_PAUSE:
            raise RuntimeError('panel pause is not active')
        if self.phase is MissionPhase.PATROL:
            if self.route.advance():
                # Position OFFBOARD remains active between forward panels.
                self.transition(PanelDistanceMissionState.MOVE_TO_PANEL)
            else:
                # Start the reverse distance pass at panel 4 immediately.
                self.phase = MissionPhase.DISTANCE
                self.transition(PanelDistanceMissionState.ARRIVE_LOITER)
        else:
            if self.route.retreat():
                # Move to the next reverse waypoint holding the current
                # distance-hold altitude instead of climbing back to cruise
                # height; distance_control was disabled and mode switched to
                # loiter_mode during DISTANCE_LOITER, so re-enter OFFBOARD
                # via the same prestream handshake used elsewhere.
                self.transition(PanelDistanceMissionState.MOVE_PRESTREAM)
            else:
                # The final reverse panel is 3 m forward of launch. Return to
                # the launch origin at the 1 m reverse-transit height before
                # handing over to AUTO.LAND.
                self.transition(
                    PanelDistanceMissionState.RETURN_HOME_PRESTREAM
                )

    def abort(self, reason: str) -> None:
        """Latch an aborted terminal state."""
        self.transition(PanelDistanceMissionState.ABORT, reason)
