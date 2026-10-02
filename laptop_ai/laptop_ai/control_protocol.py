"""Validated Pi-to-laptop mode control heartbeat."""

import ipaddress
import json
import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ControlState:
    """Latest validated Pi mode heartbeat."""

    mode: str
    active_panel_id: int
    session_id: str
    sequence: int
    received_monotonic_s: float


class ControlReceiver:
    """Accept monotonic control packets only from the configured Pi source."""

    def __init__(
        self,
        socket,
        *,
        allowed_source_id: str,
        allowed_remote_ip: str,
        timeout_s: float,
    ) -> None:
        """Configure a fail-closed receiver for one Pi address and ID."""
        if not allowed_source_id or not allowed_remote_ip or timeout_s <= 0.0:
            raise ValueError('control receiver configuration is invalid')
        self.socket = socket
        self.allowed_source_id = allowed_source_id
        self.allowed_remote_ip = str(ipaddress.ip_address(allowed_remote_ip))
        self.timeout_s = timeout_s
        self._last: Optional[ControlState] = None

    def poll(self) -> None:
        """Consume queued datagrams and retain the newest valid one."""
        while True:
            try:
                raw, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                remote_ip = str(ipaddress.ip_address(address[0]))
                if remote_ip != self.allowed_remote_ip:
                    continue
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    continue
                if payload.get('protocol_version') != 1:
                    continue
                if payload.get('source_id') != self.allowed_source_id:
                    continue
                mode = payload.get('mode')
                if mode not in {'idle', 'survey', 'clean'}:
                    continue
                sequence = payload.get('sequence')
                panel_id = payload.get('active_panel_id')
                session_id = payload.get('session_id', '')
                if isinstance(sequence, bool) or not isinstance(sequence, int):
                    continue
                if isinstance(panel_id, bool) or not isinstance(panel_id, int):
                    continue
                if not isinstance(session_id, str) or len(session_id) > 128:
                    continue
                now_s = time.monotonic()
                if self._last is not None:
                    same_session = session_id == self._last.session_id
                    legacy_fresh = (
                        not session_id
                        and same_session
                        and now_s - self._last.received_monotonic_s
                        <= self.timeout_s
                    )
                    if (
                        sequence <= self._last.sequence
                        and (session_id and same_session or legacy_fresh)
                    ):
                        continue
                self._last = ControlState(
                    mode,
                    panel_id,
                    session_id,
                    sequence,
                    now_s,
                )
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

    def state(self) -> ControlState:
        """Return idle whenever the Pi heartbeat is absent or stale."""
        now_s = time.monotonic()
        if (
            self._last is None
            or now_s - self._last.received_monotonic_s > self.timeout_s
        ):
            return ControlState('idle', -1, '', 0, now_s)
        return self._last
