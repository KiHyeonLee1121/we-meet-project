"""Pure state guards for Pixhawk-owned bounded spray pulses."""

from dataclasses import dataclass
import threading
import time
from typing import Optional


MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_CMD_DO_TRIGGER_CONTROL = 2003


@dataclass(frozen=True)
class SprayResult:
    """Result of one spray gate operation."""

    success: bool
    message: str


@dataclass(frozen=True)
class SprayStatus:
    """Immutable snapshot of the mission gate and Pixhawk pulse estimate."""

    output_enabled: bool
    session_enabled: bool
    active: bool
    request_pending: bool
    pulse_count: int
    backend: str


def pixhawk_one_shot_fields() -> dict[str, float | int | bool]:
    """Return MAVLink fields for one PX4 camera-trigger one-shot."""
    return {
        'broadcast': False,
        'command': MAV_CMD_DO_DIGICAM_CONTROL,
        'confirmation': 0,
        'param1': 0.0,
        'param2': 0.0,
        'param3': 0.0,
        'param4': 0.0,
        'param5': 1.0,
        'param6': 0.0,
        'param7': 0.0,
    }


def pixhawk_disable_trigger_fields() -> dict[str, float | int | bool]:
    """Return the PX4 trigger-disable command used as a best-effort stop."""
    return {
        'broadcast': False,
        'command': MAV_CMD_DO_TRIGGER_CONTROL,
        'confirmation': 0,
        'param1': 0.0,
        'param2': 0.0,
        'param3': 0.0,
        'param4': 0.0,
        'param5': 0.0,
        'param6': 0.0,
        'param7': 0.0,
    }


class SprayPulseGate:
    """Gate one-shot requests without pretending to observe the AUX5 pin."""

    def __init__(
        self,
        *,
        backend: str,
        output_enabled: bool,
        pulse_duration_s: float,
        minimum_pulse_s: float,
        maximum_pulse_s: float,
        cooldown_s: float,
    ) -> None:
        if backend not in {'mock', 'pixhawk'}:
            raise ValueError(f'unsupported spray backend: {backend}')
        if minimum_pulse_s <= 0.0:
            raise ValueError('minimum_pulse_s must be positive')
        if maximum_pulse_s < minimum_pulse_s:
            raise ValueError('maximum_pulse_s must not be smaller than minimum')
        if not minimum_pulse_s <= pulse_duration_s <= maximum_pulse_s:
            raise ValueError('pulse_duration_s is outside configured bounds')
        if cooldown_s < 0.0:
            raise ValueError('spray cooldown configuration is invalid')
        self.backend = backend
        self.output_enabled = bool(output_enabled)
        self.pulse_duration_s = float(pulse_duration_s)
        self.cooldown_s = float(cooldown_s)
        self._session_enabled = False
        self._request_pending = False
        self._pulse_count = 0
        self._last_pulse_started: Optional[float] = None
        self._active_until_s: Optional[float] = None
        self._lock = threading.RLock()

    def set_enabled(
        self,
        enabled: bool,
        now_s: Optional[float] = None,
    ) -> SprayResult:
        """Open or close the per-mission software gate."""
        with self._lock:
            if not enabled:
                self._session_enabled = False
                self._request_pending = False
                return SprayResult(True, 'spray session disabled')
            if not self.output_enabled:
                return SprayResult(
                    False, 'output_enabled=false blocks live spray'
                )
            if not self._session_enabled:
                self._pulse_count = 0
                if not self.is_active(now_s):
                    self._last_pulse_started = None
                    self._active_until_s = None
            self._session_enabled = True
            return SprayResult(True, 'spray session enabled')

    def begin_trigger(self, now_s: Optional[float] = None) -> SprayResult:
        """Latch exactly one request before dispatching it to MAVROS."""
        with self._lock:
            now_s = time.monotonic() if now_s is None else float(now_s)
            if not self.output_enabled:
                return SprayResult(False, 'live spray output is disabled')
            if not self._session_enabled:
                return SprayResult(False, 'spray session is not enabled')
            if self._request_pending:
                return SprayResult(False, 'spray request is already pending')
            if self.is_active(now_s):
                return SprayResult(False, 'spray pulse is already active')
            if self._last_pulse_started is not None:
                elapsed_s = now_s - self._last_pulse_started
                if elapsed_s < self.cooldown_s:
                    return SprayResult(False, 'spray cooldown is active')
            self._request_pending = True
            return SprayResult(True, 'spray request latched')

    def finish_trigger(
        self,
        accepted: bool,
        now_s: Optional[float] = None,
    ) -> SprayResult:
        """Record the PX4 ACK; active is a timer estimate, not pin feedback."""
        with self._lock:
            now_s = time.monotonic() if now_s is None else float(now_s)
            if not self._request_pending:
                return SprayResult(False, 'no spray request is pending')
            self._request_pending = False
            if not accepted:
                return SprayResult(False, 'Pixhawk rejected spray one-shot')
            self._pulse_count += 1
            self._last_pulse_started = now_s
            self._active_until_s = now_s + self.pulse_duration_s
            return SprayResult(
                True,
                f'PX4 accepted one {self.pulse_duration_s * 1000:.0f}ms '
                'trigger; physical valve movement is not sensed',
            )

    def cancel_pending(self) -> None:
        """Clear a request that failed before a Pixhawk acceptance ACK."""
        with self._lock:
            self._request_pending = False

    def stop(self, *, can_cancel_active: bool) -> SprayResult:
        """Record stop semantics without claiming unobservable pin closure."""
        with self._lock:
            self._request_pending = False
            if can_cancel_active:
                self._active_until_s = None
                return SprayResult(True, 'mock spray stopped')
            return SprayResult(
                True,
                'PX4 trigger disable sent; an active one-shot still ends at '
                'TRIG_ACT_TIME',
            )

    def is_active(self, now_s: Optional[float] = None) -> bool:
        """Return the time-based pulse estimate used only for diagnostics."""
        with self._lock:
            now_s = time.monotonic() if now_s is None else float(now_s)
            return (
                self._active_until_s is not None
                and now_s < self._active_until_s
            )

    def status(self, now_s: Optional[float] = None) -> SprayStatus:
        """Return the current software gate and estimated pulse state."""
        with self._lock:
            return SprayStatus(
                self.output_enabled,
                self._session_enabled,
                self.is_active(now_s),
                self._request_pending,
                self._pulse_count,
                self.backend,
            )
