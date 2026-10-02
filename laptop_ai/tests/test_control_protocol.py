"""Tests for fail-closed Pi control-heartbeat reception."""

import json

from laptop_ai.control_protocol import ControlReceiver


class FakeSocket:
    """Return queued UDP datagrams without opening a network socket."""

    def __init__(self, packets):
        """Store packets that recvfrom will return in order."""
        self.packets = list(packets)

    def recvfrom(self, _maximum_bytes):
        """Return one packet or emulate a non-blocking empty socket."""
        if not self.packets:
            raise BlockingIOError
        return self.packets.pop(0)


def packet(sequence=1, session_id='pi-session-1'):
    """Build one valid Pi control packet."""
    return json.dumps(
        {
            'protocol_version': 1,
            'source_id': 'pi5-01',
            'session_id': session_id,
            'mode': 'clean',
            'active_panel_id': 7,
            'sequence': sequence,
        }
    ).encode('utf-8')


def receiver(packets):
    """Build a receiver constrained to the field Pi address."""
    return ControlReceiver(
        FakeSocket(packets),
        allowed_source_id='pi5-01',
        allowed_remote_ip='192.168.1.20',
        timeout_s=1.0,
    )


def test_control_receiver_rejects_spoofed_remote_ip():
    """Packets from a different IP must not change the idle state."""
    control = receiver([(packet(), ('192.168.1.99', 5006))])
    control.poll()
    assert control.state().mode == 'idle'


def test_control_receiver_accepts_matching_id_and_remote_ip():
    """A packet matching both allowlists must update control state."""
    control = receiver([(packet(), ('192.168.1.20', 5006))])
    control.poll()
    assert control.state().mode == 'clean'
    assert control.state().active_panel_id == 7


def test_control_receiver_rejects_non_object_json():
    """Valid JSON with the wrong shape must be ignored."""
    control = receiver([(b'[]', ('192.168.1.20', 5006))])
    control.poll()
    assert control.state().mode == 'idle'


def test_control_receiver_rejects_duplicate_in_same_session():
    """A duplicate sequence in one sender session must be ignored."""
    control = receiver(
        [
            (packet(2), ('192.168.1.20', 5006)),
            (packet(2), ('192.168.1.20', 5006)),
        ]
    )
    control.poll()
    assert control.state().sequence == 2


def test_control_receiver_accepts_sequence_reset_in_new_session():
    """A Pi sender restart must establish a new monotonic sequence epoch."""
    control = receiver(
        [
            (packet(50, 'old-session'), ('192.168.1.20', 5006)),
            (packet(1, 'new-session'), ('192.168.1.20', 5006)),
        ]
    )
    control.poll()
    assert control.state().session_id == 'new-session'
    assert control.state().sequence == 1


def test_legacy_sequence_reset_requires_stale_heartbeat(monkeypatch):
    """Legacy senders may reset only after the old heartbeat is stale."""
    now = [100.0]
    monkeypatch.setattr(
        'laptop_ai.control_protocol.time.monotonic', lambda: now[0]
    )
    first_socket = FakeSocket([(packet(20, ''), ('192.168.1.20', 5006))])
    control = ControlReceiver(
        first_socket,
        allowed_source_id='pi5-01',
        allowed_remote_ip='192.168.1.20',
        timeout_s=1.0,
    )
    control.poll()
    control.socket = FakeSocket([(packet(1, ''), ('192.168.1.20', 5006))])
    control.poll()
    assert control.state().sequence == 20

    now[0] += 1.1
    control.socket = FakeSocket([(packet(1, ''), ('192.168.1.20', 5006))])
    control.poll()
    assert control.state().sequence == 1
