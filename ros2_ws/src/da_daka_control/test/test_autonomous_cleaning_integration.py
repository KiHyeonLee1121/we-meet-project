"""End-to-end pure test for a random two-panel cleaning decision flow."""

from da_daka_control.autonomous_cleaning_fsm import (
    AutonomousCleaningFsm,
    CleaningMissionState,
)
from da_daka_control.panel_mapping import PanelTarget
from da_daka_control.route_planner import plan_panel_route


def target(panel_id, east_m, north_m):
    return PanelTarget(panel_id, east_m, north_m, 1.2, 0.8, 0.9, 4)


def arrive_and_reacquire(fsm):
    fsm.panel_visible()
    fsm.transit_arrived()
    fsm.panel_reacquired()


def test_random_route_clean_skip_retry_and_home_sequence():
    surveyed = [
        target(7, 3.0, 1.0),
        target(2, 1.0, 0.0),
    ]
    route = plan_panel_route((0.0, 0.0), surveyed, (0.0, 0.0))
    assert route.panel_ids == (2, 7)

    fsm = AutonomousCleaningFsm(max_spray_attempts=3)
    fsm.start()
    fsm.precheck_complete()
    fsm.armed()
    fsm.takeoff_complete()
    fsm.survey_complete(surveyed)
    fsm.route_planned(route.panel_ids)
    assert fsm.state == CleaningMissionState.DESCEND
    fsm.descent_complete()

    # The nearest panel is already clean, so it is never sprayed.
    arrive_and_reacquire(fsm)
    fsm.cleanliness_result(False)
    assert fsm.panels[0].clean
    assert fsm.panels[0].spray_attempts == 0

    # The second panel remains dirty after one spray, then passes recheck.
    arrive_and_reacquire(fsm)
    fsm.cleanliness_result(True)
    fsm.alignment_complete()
    fsm.spray_complete()
    fsm.post_spray_alignment_complete()
    fsm.cleanliness_result(True)
    fsm.alignment_complete()
    fsm.spray_complete()
    fsm.post_spray_alignment_complete()
    fsm.cleanliness_result(False)
    assert fsm.panels[1].clean
    assert fsm.panels[1].spray_attempts == 2
    assert fsm.state == CleaningMissionState.RETURN_HOME

    fsm.home_arrived()
    fsm.landed()
    assert fsm.state == CleaningMissionState.COMPLETE
