"""Tests for the complete autonomous cleaning mission state order."""

from da_daka_control.autonomous_cleaning_fsm import (
    AutonomousCleaningFsm,
    CleaningMissionState,
)
from da_daka_control.panel_mapping import PanelTarget


def panel(panel_id):
    return PanelTarget(panel_id, float(panel_id), 0.0, 1.0, 1.0, 0.9, 3)


def reach_first_panel(fsm, panels=None):
    targets = panels or [panel(1), panel(2)]
    fsm.start()
    fsm.precheck_complete()
    fsm.armed()
    fsm.takeoff_complete()
    fsm.survey_complete(targets)
    fsm.route_planned([target.panel_id for target in targets])
    assert fsm.state == CleaningMissionState.DESCEND
    fsm.descent_complete()
    fsm.panel_visible()
    assert fsm.state == CleaningMissionState.SLOW_APPROACH
    fsm.transit_arrived()
    fsm.panel_reacquired()


def spray_and_realign(fsm):
    fsm.alignment_complete()
    assert fsm.state == CleaningMissionState.SPRAY
    fsm.spray_complete()
    assert fsm.state == CleaningMissionState.POST_SPRAY_ALIGN
    fsm.post_spray_alignment_complete()
    assert fsm.state == CleaningMissionState.VERIFY


def test_clean_panel_is_skipped_without_spray():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.cleanliness_result(False)
    assert fsm.state == CleaningMissionState.TRANSIT
    assert fsm.panels[0].clean
    assert fsm.panels[0].spray_attempts == 0
    assert fsm.current_panel.target.panel_id == 2


def test_first_spray_clean_advances_to_next_panel():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(False)
    assert fsm.state == CleaningMissionState.TRANSIT
    assert fsm.panels[0].clean
    assert fsm.panels[0].spray_attempts == 1


def test_first_spray_dirty_second_spray_clean():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    assert fsm.state == CleaningMissionState.PRECISION_ALIGN
    spray_and_realign(fsm)
    fsm.cleanliness_result(False)
    assert fsm.state == CleaningMissionState.TRANSIT
    assert fsm.panels[0].clean
    assert fsm.panels[0].spray_attempts == 2


def test_first_two_sprays_dirty_third_spray_clean():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    assert fsm.state == CleaningMissionState.PRECISION_ALIGN
    spray_and_realign(fsm)
    fsm.cleanliness_result(False)
    assert fsm.state == CleaningMissionState.TRANSIT
    assert fsm.panels[0].clean
    assert fsm.panels[0].spray_attempts == 3


def test_three_dirty_results_mark_cleaning_failed_and_advance():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    failed = fsm.panels[0]
    assert fsm.state == CleaningMissionState.TRANSIT
    assert failed.cleaning_failed
    assert not failed.clean
    assert 'remained dirty after 3 sprays' in failed.failure_reason
    assert fsm.current_panel.target.panel_id == 2


def test_last_panel_failure_still_returns_home_lands_and_completes():
    only_panel = [panel(1)]
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm, only_panel)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    spray_and_realign(fsm)
    fsm.cleanliness_result(True)
    assert fsm.panels[0].cleaning_failed
    assert fsm.state == CleaningMissionState.RETURN_HOME
    fsm.home_arrived()
    assert fsm.state == CleaningMissionState.LAND
    fsm.landed()
    assert fsm.state == CleaningMissionState.COMPLETE


def test_safety_failure_still_aborts_the_mission():
    fsm = AutonomousCleaningFsm()
    reach_first_panel(fsm)
    fsm.abort('distance sensor timeout')
    assert fsm.state == CleaningMissionState.ABORT
    assert fsm.reason == 'distance sensor timeout'
