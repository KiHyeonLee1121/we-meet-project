"""Tests for mission-side timed spray and fresh-frame gating."""

from pathlib import Path

from da_daka_control.spray_sequence import (
    perception_is_newer,
    PerceptionBarrier,
    SprayCycleTracker,
)
import yaml


def test_trigger_response_does_not_complete_the_spray():
    cycle = SprayCycleTracker()
    cycle.latch_trigger(10.0)
    cycle.accept_trigger(10.01)
    assert cycle.trigger_requested
    assert cycle.pulse_completed_s is None
    assert not cycle.complete_if_elapsed(10.01, 3.0)
    assert cycle.pulse_completed_s is None


def test_three_second_pulse_completes_only_after_timer_elapsed():
    cycle = SprayCycleTracker()
    cycle.latch_trigger(20.0)
    cycle.accept_trigger(20.0)
    assert not cycle.complete_if_elapsed(22.999, 3.0)
    assert cycle.pulse_completed_s is None
    assert cycle.complete_if_elapsed(23.0, 3.0)
    assert cycle.pulse_completed_s == 23.0
    assert not cycle.complete_if_elapsed(23.1, 3.0)


def test_duplicate_trigger_is_rejected_by_latch():
    cycle = SprayCycleTracker()
    cycle.latch_trigger(1.0)
    try:
        cycle.latch_trigger(1.1)
    except RuntimeError as exc:
        assert 'already latched' in str(exc)
    else:
        raise AssertionError('duplicate trigger was not rejected')


def test_verification_requires_a_new_packet_or_frame():
    barrier = PerceptionBarrier('session-a', 41, 100)
    assert not perception_is_newer(
        session_id='session-a', sequence=41, frame_id=100, barrier=barrier
    )
    assert perception_is_newer(
        session_id='session-a', sequence=42, frame_id=100, barrier=barrier
    )
    assert perception_is_newer(
        session_id='session-a', sequence=41, frame_id=101, barrier=barrier
    )
    assert perception_is_newer(
        session_id='session-b', sequence=0, frame_id=0, barrier=barrier
    )


def test_operational_configuration_uses_three_seconds_and_three_attempts():
    config_dir = Path(__file__).parents[1] / 'config'
    spray = yaml.safe_load(
        (config_dir / 'spray_controller.yaml').read_text(encoding='utf-8')
    )['spray_controller']['ros__parameters']
    mission = yaml.safe_load(
        (config_dir / 'autonomous_cleaning.yaml').read_text(encoding='utf-8')
    )['/**']['ros__parameters']

    assert spray['pulse_duration_s'] == 3.0
    assert spray['maximum_pulse_s'] >= 3.0
    assert spray['backend'] == 'mock'
    assert spray['mavros_command_service'] == '/mavros/cmd/command'
    assert 'gpio_chip' not in spray
    assert 'gpio_line_offset' not in spray
    assert 'maximum_pulses_per_session' not in spray
    assert mission['max_spray_attempts'] == 3
    assert mission['spray_duration_s'] == 3.0
    assert 'verification_settle_s' not in mission
    assert 'maximum_spray_panels_per_mission' not in mission
