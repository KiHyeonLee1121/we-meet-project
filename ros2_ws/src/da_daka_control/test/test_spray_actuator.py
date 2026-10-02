"""Tests for Pixhawk spray command fields and software request gates."""

from da_daka_control.spray_actuator import (
    MAV_CMD_DO_DIGICAM_CONTROL,
    MAV_CMD_DO_TRIGGER_CONTROL,
    pixhawk_disable_trigger_fields,
    pixhawk_one_shot_fields,
    SprayPulseGate,
)


def make_gate(**updates):
    values = {
        'backend': 'pixhawk',
        'output_enabled': True,
        'pulse_duration_s': 3.0,
        'minimum_pulse_s': 0.05,
        'maximum_pulse_s': 3.0,
        'cooldown_s': 0.0,
    }
    values.update(updates)
    return SprayPulseGate(**values)


def test_spray_requires_session_enable():
    gate = make_gate()
    assert not gate.begin_trigger(0.0).success
    assert not gate.status(0.0).active


def test_output_gate_cannot_be_bypassed():
    gate = make_gate(output_enabled=False)
    assert not gate.set_enabled(True).success
    assert not gate.begin_trigger(0.0).success


def test_pixhawk_one_shot_uses_camera_trigger_command():
    fields = pixhawk_one_shot_fields()
    assert fields['command'] == MAV_CMD_DO_DIGICAM_CONTROL == 203
    assert fields['param5'] == 1.0
    assert fields['broadcast'] is False


def test_accepted_one_shot_is_estimated_active_for_three_seconds():
    gate = make_gate()
    assert gate.set_enabled(True).success
    assert gate.begin_trigger(10.0).success
    assert gate.finish_trigger(True, 10.1).success
    assert gate.status(13.099).active
    assert not gate.status(13.1).active


def test_duplicate_trigger_is_blocked_while_request_or_pulse_is_active():
    gate = make_gate()
    assert gate.set_enabled(True).success
    assert gate.begin_trigger(1.0).success
    assert not gate.begin_trigger(1.1).success
    assert gate.finish_trigger(True, 1.2).success
    assert not gate.begin_trigger(2.0).success


def test_more_than_three_pulses_are_not_blocked_by_a_session_limit():
    gate = make_gate()
    assert gate.set_enabled(True).success
    for index in range(4):
        started_s = index * 4.0
        assert gate.begin_trigger(started_s).success
        assert gate.finish_trigger(True, started_s).success
    assert gate.status(20.0).pulse_count == 4


def test_stop_command_is_disable_but_does_not_claim_one_shot_cancelled():
    fields = pixhawk_disable_trigger_fields()
    assert fields['command'] == MAV_CMD_DO_TRIGGER_CONTROL == 2003
    assert fields['param1'] == 0.0

    gate = make_gate()
    gate.set_enabled(True, 0.0)
    gate.begin_trigger(0.0)
    gate.finish_trigger(True, 0.0)
    result = gate.stop(can_cancel_active=False)
    assert result.success
    assert 'TRIG_ACT_TIME' in result.message
    assert gate.status(1.0).active


def test_mock_stop_clears_estimated_activity_immediately():
    gate = make_gate(backend='mock')
    gate.set_enabled(True)
    gate.begin_trigger(0.0)
    gate.finish_trigger(True, 0.0)
    assert gate.stop(can_cancel_active=True).success
    assert not gate.status(0.1).active


def test_disable_reenable_cannot_bypass_an_active_pixhawk_pulse():
    gate = make_gate()
    gate.set_enabled(True)
    gate.begin_trigger(0.0)
    gate.finish_trigger(True, 0.0)
    gate.set_enabled(False, 0.5)
    gate.set_enabled(True, 0.5)
    assert not gate.begin_trigger(1.0).success
