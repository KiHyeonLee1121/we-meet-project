"""Pure state machine: hover at a fixed height, spray once, settle, land."""

from enum import auto, Enum
import math
from typing import Optional


class SprayHoverState(Enum):
    """States used by the standalone airborne spray-reaction test."""

    IDLE = auto()
    PRECHECK = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER_STABILISE = auto()
    SPRAY_ARM = auto()
    SPRAY = auto()
    SETTLE = auto()
    LAND = auto()
    WAIT_DISARM = auto()
    COMPLETE = auto()
    ABORT = auto()


class StableWindow:
    """Require a condition to hold continuously for a whole duration."""

    def __init__(self, duration_s: float) -> None:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError('duration_s must be finite and positive')
        self.duration_s = duration_s
        self._entered_s: Optional[float] = None

    def reset(self) -> None:
        """Drop any partially accumulated stable time."""
        self._entered_s = None

    def elapsed_s(self, now_s: float) -> float:
        """Return how long the condition has held, or zero."""
        if self._entered_s is None:
            return 0.0
        return max(0.0, now_s - self._entered_s)

    def update(self, holding: bool, now_s: float) -> bool:
        """Return True once the condition has held for the full duration."""
        if not math.isfinite(now_s):
            raise ValueError('now_s must be finite')
        if not holding:
            self._entered_s = None
            return False
        if self._entered_s is None:
            self._entered_s = now_s
            return False
        return now_s - self._entered_s >= self.duration_s


def vertical_hold_speed(
    height_error_m: float,
    gain_per_s: float,
    maximum_speed_mps: float,
) -> float:
    """Return a bounded proportional climb command for one height error."""
    values = (height_error_m, gain_per_s, maximum_speed_mps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('vertical hold inputs must be finite')
    if gain_per_s < 0.0:
        raise ValueError('gain_per_s must be non-negative')
    if maximum_speed_mps <= 0.0:
        raise ValueError('maximum_speed_mps must be positive')
    command = gain_per_s * height_error_m
    return max(-maximum_speed_mps, min(maximum_speed_mps, command))


def horizontal_hold_speed(
    position_error_m: float,
    gain_per_s: float,
    maximum_speed_mps: float,
) -> float:
    """Return a bounded proportional command holding one horizontal axis."""
    values = (position_error_m, gain_per_s, maximum_speed_mps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('horizontal hold inputs must be finite')
    if gain_per_s < 0.0:
        raise ValueError('gain_per_s must be non-negative')
    if maximum_speed_mps <= 0.0:
        raise ValueError('maximum_speed_mps must be positive')
    command = gain_per_s * position_error_m
    return max(-maximum_speed_mps, min(maximum_speed_mps, command))


class SprayHoverTestFsm:
    """Enforce the exact airborne spray-test order without performing I/O."""

    def __init__(self) -> None:
        self.state = SprayHoverState.IDLE
        self.reason = 'IDLE'
        self.spray_fired = False

    @property
    def active(self) -> bool:
        """Report whether the test currently owns the aircraft."""
        return self.state not in {
            SprayHoverState.IDLE,
            SprayHoverState.COMPLETE,
            SprayHoverState.ABORT,
        }

    @property
    def airborne(self) -> bool:
        """Report whether the aircraft is expected to be off the ground."""
        return self.state in {
            SprayHoverState.TAKEOFF,
            SprayHoverState.HOVER_STABILISE,
            SprayHoverState.SPRAY_ARM,
            SprayHoverState.SPRAY,
            SprayHoverState.SETTLE,
            SprayHoverState.LAND,
        }

    def transition(self, state: SprayHoverState, reason: str = '') -> None:
        """Move to one new state and record why."""
        self.state = state
        self.reason = reason or state.name

    def _require(self, *expected: SprayHoverState) -> None:
        if self.state not in expected:
            names = '/'.join(item.name for item in expected)
            raise RuntimeError(
                f'expected {names} but the test is in {self.state.name}'
            )

    def start(self) -> None:
        """Begin one test run from the idle state."""
        self._require(SprayHoverState.IDLE)
        self.spray_fired = False
        self.transition(SprayHoverState.PRECHECK, 'test requested')

    def precheck_complete(self) -> None:
        """Accept ground readiness and request arming."""
        self._require(SprayHoverState.PRECHECK)
        self.transition(SprayHoverState.ARMING)

    def armed(self) -> None:
        """Accept motor arming and begin the climb."""
        self._require(SprayHoverState.ARMING)
        self.transition(SprayHoverState.TAKEOFF)

    def takeoff_complete(self) -> None:
        """Accept target height and begin the pre-spray settling window."""
        self._require(SprayHoverState.TAKEOFF)
        self.transition(SprayHoverState.HOVER_STABILISE)

    def hover_stable(self) -> None:
        """Accept a settled hover and enable the spray path."""
        self._require(SprayHoverState.HOVER_STABILISE)
        self.transition(SprayHoverState.SPRAY_ARM, 'hover settled')

    def spray_path_ready(self) -> None:
        """Accept enabled valve/feedforward and release one pulse."""
        self._require(SprayHoverState.SPRAY_ARM)
        self.transition(SprayHoverState.SPRAY, 'valve and feedforward enabled')

    def spray_complete(self) -> None:
        """Count the elapsed pulse and begin post-spray settling."""
        self._require(SprayHoverState.SPRAY)
        self.spray_fired = True
        self.transition(SprayHoverState.SETTLE, 'pulse elapsed')

    def settled(self) -> None:
        """Accept a recovered hover and begin the landing."""
        self._require(SprayHoverState.SETTLE)
        self.transition(SprayHoverState.LAND, 'hover recovered after spray')

    def landing_started(self) -> None:
        """Record that the landing command has been accepted."""
        self._require(SprayHoverState.LAND)
        self.transition(SprayHoverState.WAIT_DISARM)

    def disarmed(self) -> None:
        """Complete the test after touchdown and disarm."""
        self._require(SprayHoverState.WAIT_DISARM)
        self.transition(SprayHoverState.COMPLETE, 'test complete')

    def abort(self, reason: str) -> None:
        """Abandon the test for any reason from any state."""
        if not reason:
            raise ValueError('abort requires a reason')
        self.transition(SprayHoverState.ABORT, reason)
