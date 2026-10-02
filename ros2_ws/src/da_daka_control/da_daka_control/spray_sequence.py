"""Pure helpers for timed spray completion and fresh image barriers."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PerceptionBarrier:
    """Identify the newest perception result at assumed pulse completion."""

    session_id: str
    sequence: int
    frame_id: int


def perception_is_newer(
    *,
    session_id: str,
    sequence: int,
    frame_id: int,
    barrier: Optional[PerceptionBarrier],
) -> bool:
    """Return whether a packet/frame is newer than the closure barrier."""
    if barrier is None:
        return True
    if session_id != barrier.session_id:
        return True
    return sequence > barrier.sequence or frame_id > barrier.frame_id


class SprayCycleTracker:
    """Latch one trigger and complete after its accepted pulse duration."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Prepare for exactly one new pulse request."""
        self.trigger_requested = False
        self.trigger_requested_s: Optional[float] = None
        self.trigger_accepted_s: Optional[float] = None
        self.pulse_completed_s: Optional[float] = None

    def latch_trigger(self, now_s: float) -> None:
        """Record dispatch of one trigger and reject accidental duplicates."""
        if self.trigger_requested:
            raise RuntimeError('spray trigger is already latched')
        self.trigger_requested = True
        self.trigger_requested_s = float(now_s)

    def accept_trigger(self, now_s: float) -> None:
        """Record the successful trigger response used to start the wait."""
        if not self.trigger_requested:
            raise RuntimeError('spray trigger was not requested')
        if self.trigger_accepted_s is not None:
            raise RuntimeError('spray trigger was already accepted')
        self.trigger_accepted_s = float(now_s)

    def complete_if_elapsed(self, now_s: float, duration_s: float) -> bool:
        """Return True once after the accepted trigger duration has elapsed."""
        if duration_s <= 0.0:
            raise ValueError('spray duration must be positive')
        if self.trigger_accepted_s is None or self.pulse_completed_s is not None:
            return False
        completion_s = self.trigger_accepted_s + float(duration_s)
        if now_s < completion_s:
            return False
        self.pulse_completed_s = completion_s
        return True
