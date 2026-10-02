"""Pure multi-panel cleaning mission state machine."""

from dataclasses import dataclass, field
from enum import auto, Enum
from typing import Iterable, Optional

from da_daka_control.panel_mapping import PanelTarget


class CleaningMissionState(Enum):
    """Complete scan, clean, verify, return and land mission states."""

    IDLE = auto()
    PRECHECK = auto()
    ARMING = auto()
    TAKEOFF = auto()
    SURVEY = auto()
    PLAN_ROUTE = auto()
    DESCEND = auto()
    TRANSIT = auto()
    SLOW_APPROACH = auto()
    REACQUIRE = auto()
    ASSESS = auto()
    PRECISION_ALIGN = auto()
    SPRAY = auto()
    POST_SPRAY_ALIGN = auto()
    VERIFY = auto()
    RETURN_HOME = auto()
    LAND = auto()
    COMPLETE = auto()
    ABORT = auto()


@dataclass
class PanelProgress:
    """Per-panel mission result retained across retries."""

    target: PanelTarget
    spray_attempts: int = 0
    clean: bool = False
    cleaning_failed: bool = False
    failure_reason: str = ''
    awaiting_verification: bool = False


@dataclass
class AutonomousCleaningFsm:
    """Enforce the exact high-level mission order without performing I/O."""

    max_spray_attempts: int = 3
    state: CleaningMissionState = CleaningMissionState.IDLE
    reason: str = 'IDLE'
    panels: list[PanelProgress] = field(default_factory=list)
    current_index: int = 0

    def __post_init__(self) -> None:
        if self.max_spray_attempts != 3:
            raise ValueError('max_spray_attempts must be exactly 3')

    @property
    def active(self) -> bool:
        """Return whether the mission still owns an active sequence."""
        return self.state not in {
            CleaningMissionState.IDLE,
            CleaningMissionState.COMPLETE,
            CleaningMissionState.ABORT,
        }

    @property
    def current_panel(self) -> Optional[PanelProgress]:
        """Return the current panel progress or None after the route."""
        if 0 <= self.current_index < len(self.panels):
            return self.panels[self.current_index]
        return None

    def transition(self, state: CleaningMissionState, reason: str = '') -> None:
        """Move to one explicit mission state and record its reason."""
        self.state = state
        self.reason = reason or state.name

    def start(self) -> None:
        """Reset prior progress and enter precheck."""
        if self.active:
            raise RuntimeError('mission is already active')
        self.panels = []
        self.current_index = 0
        self.transition(CleaningMissionState.PRECHECK, 'mission requested')

    def precheck_complete(self) -> None:
        """Advance from precheck to onboard arming."""
        self._require(CleaningMissionState.PRECHECK)
        self.transition(CleaningMissionState.ARMING)

    def armed(self) -> None:
        """Advance from arming to the 3 m takeoff."""
        self._require(CleaningMissionState.ARMING)
        self.transition(CleaningMissionState.TAKEOFF)

    def takeoff_complete(self) -> None:
        """Begin the high-altitude panel survey."""
        self._require(CleaningMissionState.TAKEOFF)
        self.transition(CleaningMissionState.SURVEY)

    def survey_complete(self, panels: Iterable[PanelTarget]) -> None:
        """Store stable metric panel targets and request route planning."""
        self._require(CleaningMissionState.SURVEY)
        targets = tuple(panels)
        if not targets:
            raise ValueError('survey must find at least one panel')
        self.panels = [PanelProgress(target) for target in targets]
        self.transition(CleaningMissionState.PLAN_ROUTE)

    def route_planned(self, ordered_panel_ids: Iterable[int]) -> None:
        """Apply a complete permutation of the surveyed panel IDs."""
        self._require(CleaningMissionState.PLAN_ROUTE)
        ordered_ids = tuple(ordered_panel_ids)
        by_id = {progress.target.panel_id: progress for progress in self.panels}
        if len(by_id) != len(self.panels):
            raise ValueError('survey contains duplicate panel IDs')
        if len(ordered_ids) != len(by_id) or set(ordered_ids) != set(by_id):
            raise ValueError('route must contain every surveyed panel exactly once')
        self.panels = [by_id[panel_id] for panel_id in ordered_ids]
        self.current_index = 0
        self.transition(CleaningMissionState.DESCEND)

    def descent_complete(self) -> None:
        """Begin transit after reaching spray height."""
        self._require(CleaningMissionState.DESCEND)
        self.transition(CleaningMissionState.TRANSIT)

    def panel_visible(self) -> None:
        """Slow transit once a panel enters the camera frame."""
        if self.state == CleaningMissionState.TRANSIT:
            self.transition(
                CleaningMissionState.SLOW_APPROACH,
                'panel entered camera frame',
            )

    def transit_arrived(self) -> None:
        """Begin local target reacquisition at a metric panel target."""
        if self.state not in {
            CleaningMissionState.TRANSIT,
            CleaningMissionState.SLOW_APPROACH,
        }:
            raise RuntimeError('arrival is valid only during panel transit')
        self.transition(CleaningMissionState.REACQUIRE)

    def panel_reacquired(self) -> None:
        """Begin the initial clean/dirty assessment."""
        self._require(CleaningMissionState.REACQUIRE)
        panel = self._require_current_panel()
        if panel.awaiting_verification:
            self.transition(
                CleaningMissionState.POST_SPRAY_ALIGN,
                'reacquired panel requires post-spray alignment',
            )
        else:
            self.transition(CleaningMissionState.ASSESS)

    def cleanliness_result(self, dirt_found: bool) -> None:
        """Skip a clean panel or request alignment for a dirty panel."""
        if self.state not in {
            CleaningMissionState.ASSESS,
            CleaningMissionState.VERIFY,
        }:
            raise RuntimeError('cleanliness result is not currently expected')
        panel = self._require_current_panel()
        if self.state == CleaningMissionState.VERIFY:
            panel.awaiting_verification = False
        if not dirt_found:
            panel.clean = True
            self._advance_panel()
            return
        if panel.spray_attempts >= self.max_spray_attempts:
            panel.cleaning_failed = True
            panel.failure_reason = (
                f'panel {panel.target.panel_id} remained dirty after '
                f'{panel.spray_attempts} sprays'
            )
            self._advance_panel()
            return
        self.transition(
            CleaningMissionState.PRECISION_ALIGN,
            'dirt requires nozzle/distance/heading alignment',
        )

    def alignment_complete(self) -> None:
        """Permit one spray only after precision alignment."""
        self._require(CleaningMissionState.PRECISION_ALIGN)
        self.transition(CleaningMissionState.SPRAY)

    def spray_complete(self) -> None:
        """Count one elapsed pulse and require immediate realignment."""
        self._require(CleaningMissionState.SPRAY)
        panel = self._require_current_panel()
        panel.spray_attempts += 1
        panel.awaiting_verification = True
        self.transition(
            CleaningMissionState.POST_SPRAY_ALIGN,
            'three-second pulse elapsed; realign before fresh verification',
        )

    def post_spray_alignment_complete(self) -> None:
        """Permit verification after fresh-data precision alignment."""
        self._require(CleaningMissionState.POST_SPRAY_ALIGN)
        self.transition(CleaningMissionState.VERIFY)

    def target_lost(self) -> None:
        """Return to bounded panel reacquisition after vision loss."""
        if self.state not in {
            CleaningMissionState.SLOW_APPROACH,
            CleaningMissionState.ASSESS,
            CleaningMissionState.PRECISION_ALIGN,
            CleaningMissionState.POST_SPRAY_ALIGN,
            CleaningMissionState.VERIFY,
        }:
            raise RuntimeError('target loss is invalid in the current state')
        self.transition(CleaningMissionState.REACQUIRE, 'panel target lost')

    def home_arrived(self) -> None:
        """Request landing after the launch XY is reached."""
        self._require(CleaningMissionState.RETURN_HOME)
        self.transition(CleaningMissionState.LAND)

    def landed(self) -> None:
        """Complete the mission after confirmed disarm on ground."""
        self._require(CleaningMissionState.LAND)
        self.transition(CleaningMissionState.COMPLETE, 'mission complete')

    def abort(self, reason: str) -> None:
        """Latch an unrecoverable mission failure reason."""
        if not reason:
            raise ValueError('abort reason cannot be empty')
        self.transition(CleaningMissionState.ABORT, reason)

    def _advance_panel(self) -> None:
        self.current_index += 1
        if self.current_index >= len(self.panels):
            self.transition(CleaningMissionState.RETURN_HOME)
        else:
            self.transition(CleaningMissionState.TRANSIT)

    def _require(self, state: CleaningMissionState) -> None:
        if self.state != state:
            raise RuntimeError(
                f'expected {state.name}, current state is {self.state.name}'
            )

    def _require_current_panel(self) -> PanelProgress:
        panel = self.current_panel
        if panel is None:
            raise RuntimeError('mission has no current panel')
        return panel
