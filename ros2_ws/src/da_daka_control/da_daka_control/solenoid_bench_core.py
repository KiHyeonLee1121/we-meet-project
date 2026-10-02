"""Pure safety checks for the ground-only solenoid pulse node."""

from dataclasses import dataclass
from typing import Optional


MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_RESULT_ACCEPTED = 0
LANDED_STATE_ON_GROUND = 1


@dataclass(frozen=True)
class BenchSnapshot:
    """State used to decide whether one bench pulse may be sent."""

    now_s: float
    configuration_approved: bool
    connected: Optional[bool]
    armed: Optional[bool]
    landed_state: Optional[int]
    state_time_s: Optional[float]
    extended_state_time_s: Optional[float]
    command_pending: bool
    attempts: int
    last_attempt_time_s: Optional[float]


def bench_interlock_failures(
    snapshot: BenchSnapshot,
    telemetry_timeout_s: float,
    cooldown_s: float,
    max_pulses_per_session: int,
) -> list[str]:
    """Return every reason that blocks a ground-only pulse."""
    failures: list[str] = []
    if not snapshot.configuration_approved:
        failures.append('bench_test_approved is false')
    if snapshot.command_pending:
        failures.append('a trigger command is already pending')
    if (
        snapshot.state_time_s is None
        or snapshot.now_s - snapshot.state_time_s > telemetry_timeout_s
    ):
        failures.append('MAVROS state is unavailable or stale')
    else:
        if snapshot.connected is not True:
            failures.append('Pixhawk is not connected')
        if snapshot.armed is not False:
            failures.append('vehicle is not confirmed disarmed')
    if (
        snapshot.extended_state_time_s is None
        or snapshot.now_s - snapshot.extended_state_time_s
        > telemetry_timeout_s
    ):
        failures.append('MAVROS extended state is unavailable or stale')
    elif snapshot.landed_state != LANDED_STATE_ON_GROUND:
        failures.append('vehicle is not confirmed on ground')
    if snapshot.attempts >= max_pulses_per_session:
        failures.append('session pulse-attempt limit reached')
    if snapshot.last_attempt_time_s is not None:
        elapsed_s = snapshot.now_s - snapshot.last_attempt_time_s
        if elapsed_s < cooldown_s:
            failures.append(
                f'cooldown active for {cooldown_s - elapsed_s:.1f}s'
            )
    return failures


def digicam_command_parameters() -> tuple[float, ...]:
    """Return MAV_CMD_DO_DIGICAM_CONTROL parameters for one trigger pulse."""
    return (0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
