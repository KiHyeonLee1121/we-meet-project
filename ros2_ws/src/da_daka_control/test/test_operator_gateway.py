"""Tests for the local-only operator gateway safety defaults."""

import json
from pathlib import Path
import socket

from da_daka_control.operator_gateway_node import (
    OperatorGatewayNode,
    VALIDATION_CHECKLIST,
)
import rclpy


def socket_request(path: Path, payload: dict) -> dict:
    """Send one newline-delimited JSON request to the test gateway."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(1.0)
        client.connect(str(path))
        client.sendall(json.dumps(payload).encode('utf-8') + b'\n')
        response = client.recv(65536)
    return json.loads(response.decode('utf-8'))


def test_gateway_defaults_to_status_only_and_rejects_unknown_commands(tmp_path):
    socket_path = tmp_path / 'operator.sock'
    rclpy.init(
        args=[
            '--ros-args',
            '-p',
            f'socket_path:={socket_path}',
        ]
    )
    node = OperatorGatewayNode()
    try:
        response = socket_request(socket_path, {'command': 'status'})
        assert response['ok']
        assert response['status']['gateway_online']
        assert not response['status']['operator_start_enabled']
        assert not response['status']['start_allowed']
        assert not response['status']['validation_start_enabled']
        assert not response['status']['validation_start_allowed']

        rejected = socket_request(socket_path, {'command': 'arming'})
        assert not rejected['ok']
        assert rejected['error'] == 'command is not allowed'

        start = socket_request(
            socket_path,
            {
                'command': 'start',
                'confirmation': 'START AUTONOMOUS CLEANING',
            },
        )
        assert not start['ok']
        assert 'locked' in start['error']

        validation = socket_request(
            socket_path,
            {
                'command': 'validation_start',
                'confirmation': 'START 1M FLIGHT VALIDATION',
                'checklist': sorted(VALIDATION_CHECKLIST),
            },
        )
        assert not validation['ok']
        assert 'locked' in validation['error']
    finally:
        node.destroy_node()
        rclpy.shutdown()
