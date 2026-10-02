"""Expose a narrow local-only operator interface for the cleaning mission."""

import json
import math
import os
from pathlib import Path
import queue
import socketserver
import stat
import threading
import time
from typing import Optional

from mavros_msgs.msg import ExtendedState, State
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger


START_CONFIRMATION = 'START AUTONOMOUS CLEANING'
VALIDATION_CONFIRMATION = 'START 1M FLIGHT VALIDATION'
LOCALIZATION_TEST_CONFIRMATION = 'START PANEL LOCALIZATION TEST'
VALIDATION_CHECKLIST = {
    'flight_area_clear',
    'tether_installed',
    'propellers_inspected',
    'qgc_emergency_ready',
    'observer_ready',
    'spray_power_isolated',
}
LOCALIZATION_TEST_CHECKLIST = {
    'flight_area_clear',
    'tether_installed',
    'propellers_inspected',
    'qgc_emergency_ready',
    'observer_ready',
    'camera_ai_ready',
    'two_panel_layout_confirmed',
    'spray_power_isolated',
}
INACTIVE_STATES = {'IDLE', 'COMPLETE', 'ABORT'}


class PendingCommand:
    """Bridge one socket request into the ROS executor thread."""

    def __init__(self, command: str) -> None:
        self.command = command
        self.done = threading.Event()
        self.result: dict = {}


class OperatorRequestHandler(socketserver.StreamRequestHandler):
    """Serve one newline-delimited JSON request on the local socket."""

    def handle(self) -> None:
        line = self.rfile.readline(16385)
        if not line:
            return
        if len(line) > 16384 or not line.endswith(b'\n'):
            result = {'ok': False, 'error': 'request is too large'}
        else:
            try:
                request = json.loads(line.decode('utf-8'))
                if not isinstance(request, dict):
                    raise ValueError('request must be a JSON object')
                result = self.server.gateway.handle_socket_request(request)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                result = {'ok': False, 'error': str(exc)}
        encoded = json.dumps(result, separators=(',', ':')).encode('utf-8')
        self.wfile.write(encoded + b'\n')


class OperatorSocketServer(
    socketserver.ThreadingMixIn,
    socketserver.UnixStreamServer,
):
    """Run independent local request threads without exposing a TCP port."""

    daemon_threads = True

    def __init__(self, path: str, gateway) -> None:
        self.gateway = gateway
        super().__init__(path, OperatorRequestHandler)


class OperatorGatewayNode(Node):
    """Aggregate status and allow only mission start and abort requests."""

    ON_GROUND = 1

    def __init__(self) -> None:
        super().__init__('operator_gateway')
        self.declare_parameter(
            'socket_path', '/workspace/run/operator_gateway.sock'
        )
        self.declare_parameter('operator_start_enabled', False)
        self.declare_parameter('validation_start_enabled', False)
        self.declare_parameter('localization_test_start_enabled', False)
        self.declare_parameter('command_timeout_s', 5.0)
        self._socket_path = str(self.get_parameter('socket_path').value)
        self._operator_start_enabled = bool(
            self.get_parameter('operator_start_enabled').value
        )
        self._validation_start_enabled = bool(
            self.get_parameter('validation_start_enabled').value
        )
        self._localization_test_start_enabled = bool(
            self.get_parameter('localization_test_start_enabled').value
        )
        self._command_timeout_s = float(
            self.get_parameter('command_timeout_s').value
        )
        if self._command_timeout_s <= 0.0:
            raise ValueError('command_timeout_s must be positive')

        self._lock = threading.Lock()
        self._commands: queue.Queue[PendingCommand] = queue.Queue(maxsize=8)
        self._snapshot = self._initial_snapshot()
        self._readiness_received_s: Optional[float] = None
        self._validation_readiness_received_s: Optional[float] = None
        self._localization_test_readiness_received_s: Optional[float] = None
        self._start_client = self.create_client(
            Trigger, '/autonomous_cleaning/start'
        )
        self._abort_client = self.create_client(
            Trigger, '/autonomous_cleaning/abort'
        )
        self._validation_start_client = self.create_client(
            Trigger, '/mission/start'
        )
        self._validation_abort_client = self.create_client(
            Trigger, '/mission/abort'
        )
        self._localization_test_start_client = self.create_client(
            Trigger, '/panel_localization_test/start'
        )
        self._localization_test_abort_client = self.create_client(
            Trigger, '/panel_localization_test/abort'
        )
        self._create_subscriptions()
        self.create_timer(0.05, self._process_commands)
        self._server = self._start_socket_server()
        self.get_logger().info(
            f'Operator gateway listening on {self._socket_path}; '
            f'operator_start_enabled={self._operator_start_enabled}; '
            f'validation_start_enabled={self._validation_start_enabled}'
        )

    def _initial_snapshot(self) -> dict:
        return {
            'mission_state': 'UNKNOWN',
            'mission_result': '',
            'current_panel_id': -1,
            'mavros_connected': False,
            'armed': False,
            'flight_mode': '',
            'landed_state': None,
            'battery_percent': None,
            'lidar_m': None,
            'ai_healthy': False,
            'altitude_guard_triggered': False,
            'spray_backend': None,
            'spray_output_enabled': None,
            'spray_session_enabled': None,
            'readiness': {
                'ready': False,
                'failures': ['mission readiness has not been received'],
            },
            'validation_state': 'UNKNOWN',
            'validation_result': '',
            'validation_readiness': {
                'ready': False,
                'failures': ['validation readiness has not been received'],
            },
            'localization_test_state': 'UNKNOWN',
            'localization_test_result': '',
            'localization_test_panel_id': -1,
            'localization_test_readiness': {
                'ready': False,
                'failures': [
                    'localization test readiness has not been received'
                ],
            },
        }

    def _create_subscriptions(self) -> None:
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/autonomous_cleaning/state', self._state_cb, latched
        )
        self.create_subscription(
            String, '/autonomous_cleaning/result', self._result_cb, latched
        )
        self.create_subscription(
            String,
            '/autonomous_cleaning/readiness',
            self._readiness_cb,
            latched,
        )
        self.create_subscription(
            String, '/mission/state', self._validation_state_cb, latched
        )
        self.create_subscription(
            String, '/mission/result', self._validation_result_cb, latched
        )
        self.create_subscription(
            String,
            '/mission/readiness',
            self._validation_readiness_cb,
            latched,
        )
        self.create_subscription(
            String,
            '/panel_localization_test/state',
            self._localization_test_state_cb,
            latched,
        )
        self.create_subscription(
            String,
            '/panel_localization_test/result',
            self._localization_test_result_cb,
            latched,
        )
        self.create_subscription(
            String,
            '/panel_localization_test/readiness',
            self._localization_test_readiness_cb,
            latched,
        )
        self.create_subscription(
            Int32,
            '/autonomous_cleaning/current_panel_id',
            self._panel_cb,
            latched,
        )
        self.create_subscription(
            Int32,
            '/panel_localization_test/current_panel_id',
            self._localization_test_panel_cb,
            latched,
        )
        self.create_subscription(State, '/mavros/state', self._mavros_cb, 10)
        self.create_subscription(
            ExtendedState, '/mavros/extended_state', self._extended_cb, 10
        )
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range, '/distance/filtered', self._range_cb, qos_profile_sensor_data
        )
        self.create_subscription(Bool, '/ai/health', self._ai_cb, latched)
        self.create_subscription(
            Bool,
            '/altitude_guard/triggered',
            self._altitude_guard_cb,
            latched,
        )
        self.create_subscription(String, '/spray/status', self._spray_cb, 10)

    def _set_values(self, **values) -> None:
        with self._lock:
            self._snapshot.update(values)

    def _state_cb(self, message: String) -> None:
        self._set_values(mission_state=str(message.data))

    def _result_cb(self, message: String) -> None:
        self._set_values(mission_result=str(message.data))

    def _readiness_cb(self, message: String) -> None:
        try:
            readiness = json.loads(message.data)
            if not isinstance(readiness, dict):
                raise ValueError('readiness is not an object')
            ready = bool(readiness.get('ready', False))
            failures = readiness.get('failures', [])
            if not isinstance(failures, list):
                raise ValueError('readiness failures are not a list')
            readiness['ready'] = ready
            readiness['failures'] = [str(item) for item in failures]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            readiness = {
                'ready': False,
                'failures': [f'invalid mission readiness: {exc}'],
            }
        self._readiness_received_s = time.monotonic()
        self._set_values(readiness=readiness)

    def _panel_cb(self, message: Int32) -> None:
        self._set_values(current_panel_id=int(message.data))

    def _localization_test_panel_cb(self, message: Int32) -> None:
        self._set_values(localization_test_panel_id=int(message.data))

    def _validation_state_cb(self, message: String) -> None:
        self._set_values(validation_state=str(message.data))

    def _validation_result_cb(self, message: String) -> None:
        self._set_values(validation_result=str(message.data))

    def _validation_readiness_cb(self, message: String) -> None:
        try:
            readiness = json.loads(message.data)
            if not isinstance(readiness, dict):
                raise ValueError('validation readiness is not an object')
            failures = readiness.get('failures', [])
            if not isinstance(failures, list):
                raise ValueError('validation readiness failures are not a list')
            readiness['ready'] = bool(readiness.get('ready', False))
            readiness['failures'] = [str(item) for item in failures]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            readiness = {
                'ready': False,
                'failures': [f'invalid validation readiness: {exc}'],
            }
        self._validation_readiness_received_s = time.monotonic()
        self._set_values(validation_readiness=readiness)

    def _localization_test_state_cb(self, message: String) -> None:
        self._set_values(localization_test_state=str(message.data))

    def _localization_test_result_cb(self, message: String) -> None:
        self._set_values(localization_test_result=str(message.data))

    def _localization_test_readiness_cb(self, message: String) -> None:
        try:
            readiness = json.loads(message.data)
            if not isinstance(readiness, dict):
                raise ValueError('localization test readiness is not an object')
            failures = readiness.get('failures', [])
            if not isinstance(failures, list):
                raise ValueError('localization test failures are not a list')
            readiness['ready'] = bool(readiness.get('ready', False))
            readiness['failures'] = [str(item) for item in failures]
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            readiness = {
                'ready': False,
                'failures': [f'invalid localization test readiness: {exc}'],
            }
        self._localization_test_readiness_received_s = time.monotonic()
        self._set_values(localization_test_readiness=readiness)

    def _mavros_cb(self, message: State) -> None:
        self._set_values(
            mavros_connected=bool(message.connected),
            armed=bool(message.armed),
            flight_mode=str(message.mode),
        )

    def _extended_cb(self, message: ExtendedState) -> None:
        self._set_values(landed_state=int(message.landed_state))

    def _battery_cb(self, message: BatteryState) -> None:
        value = float(message.percentage)
        percent = (
            value * 100.0
            if math.isfinite(value) and 0.0 <= value <= 1.0
            else None
        )
        self._set_values(battery_percent=percent)

    def _range_cb(self, message: Range) -> None:
        value = float(message.range)
        self._set_values(lidar_m=value if math.isfinite(value) else None)

    def _ai_cb(self, message: Bool) -> None:
        self._set_values(ai_healthy=bool(message.data))

    def _altitude_guard_cb(self, message: Bool) -> None:
        self._set_values(altitude_guard_triggered=bool(message.data))

    def _spray_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            values = {
                'spray_backend': str(payload['backend']),
                'spray_output_enabled': bool(payload['output_enabled']),
                'spray_session_enabled': bool(payload['session_enabled']),
            }
        except (json.JSONDecodeError, KeyError, TypeError):
            values = {
                'spray_backend': None,
                'spray_output_enabled': None,
                'spray_session_enabled': None,
            }
        self._set_values(**values)

    def _start_socket_server(self) -> OperatorSocketServer:
        path = Path(self._socket_path)
        path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        if path.exists() or path.is_socket():
            mode = os.stat(path, follow_symlinks=False).st_mode
            if not stat.S_ISSOCK(mode):
                raise RuntimeError(f'refusing to replace non-socket path: {path}')
            path.unlink()
        server = OperatorSocketServer(str(path), self)
        os.chmod(path, 0o666)
        thread = threading.Thread(
            target=server.serve_forever,
            name='operator-gateway-socket',
            daemon=True,
        )
        thread.start()
        return server

    def handle_socket_request(self, request: dict) -> dict:
        """Validate a local request and dispatch its allowlisted command."""
        command = request.get('command')
        if command == 'status':
            return {'ok': True, 'status': self._status_snapshot()}
        if command == 'ping':
            return {'ok': True, 'message': 'pong'}
        allowed = {
            'start', 'abort', 'validation_start', 'validation_abort',
            'localization_test_start', 'localization_test_abort',
        }
        if command not in allowed:
            return {'ok': False, 'error': 'command is not allowed'}
        if command == 'start':
            rejection = self._start_rejection(request)
            if rejection:
                return {'ok': False, 'error': rejection}
        if command == 'validation_start':
            rejection = self._validation_start_rejection(request)
            if rejection:
                return {'ok': False, 'error': rejection}
        if command == 'localization_test_start':
            rejection = self._localization_test_start_rejection(request)
            if rejection:
                return {'ok': False, 'error': rejection}
        pending = PendingCommand(command)
        try:
            self._commands.put_nowait(pending)
        except queue.Full:
            return {'ok': False, 'error': 'operator command queue is busy'}
        if not pending.done.wait(self._command_timeout_s):
            return {'ok': False, 'error': 'mission service response timed out'}
        return pending.result

    def _status_snapshot(self) -> dict:
        with self._lock:
            snapshot = json.loads(json.dumps(self._snapshot))
        readiness_age = None
        if self._readiness_received_s is not None:
            readiness_age = max(
                0.0, time.monotonic() - self._readiness_received_s
            )
        freshness_ok = readiness_age is not None and readiness_age <= 2.0
        validation_readiness_age = None
        if self._validation_readiness_received_s is not None:
            validation_readiness_age = max(
                0.0,
                time.monotonic() - self._validation_readiness_received_s,
            )
        validation_fresh = (
            validation_readiness_age is not None
            and validation_readiness_age <= 2.0
        )
        localization_test_readiness_age = None
        if self._localization_test_readiness_received_s is not None:
            localization_test_readiness_age = max(
                0.0,
                time.monotonic() - self._localization_test_readiness_received_s,
            )
        localization_test_fresh = (
            localization_test_readiness_age is not None
            and localization_test_readiness_age <= 2.0
        )
        readiness = snapshot['readiness']
        validation_readiness = snapshot['validation_readiness']
        localization_test_readiness = snapshot['localization_test_readiness']
        snapshot.update(
            {
                'gateway_online': True,
                'operator_start_enabled': self._operator_start_enabled,
                'start_service_ready': self._start_client.service_is_ready(),
                'abort_service_ready': self._abort_client.service_is_ready(),
                'readiness_age_s': readiness_age,
                'start_allowed': bool(
                    self._operator_start_enabled
                    and freshness_ok
                    and readiness.get('ready', False)
                    and snapshot['mission_state'] in INACTIVE_STATES
                    and self._start_client.service_is_ready()
                ),
                'validation_start_enabled': self._validation_start_enabled,
                'validation_start_service_ready': (
                    self._validation_start_client.service_is_ready()
                ),
                'validation_abort_service_ready': (
                    self._validation_abort_client.service_is_ready()
                ),
                'validation_readiness_age_s': validation_readiness_age,
                'validation_start_allowed': bool(
                    self._validation_start_enabled
                    and validation_fresh
                    and validation_readiness.get('ready', False)
                    and snapshot['validation_state'] in {
                        'UNKNOWN', *INACTIVE_STATES,
                    }
                    and self._validation_start_client.service_is_ready()
                    and snapshot['mission_state'] in {
                        'UNKNOWN',
                        *INACTIVE_STATES,
                    }
                ),
                'localization_test_start_enabled': (
                    self._localization_test_start_enabled
                ),
                'localization_test_start_service_ready': (
                    self._localization_test_start_client.service_is_ready()
                ),
                'localization_test_abort_service_ready': (
                    self._localization_test_abort_client.service_is_ready()
                ),
                'localization_test_readiness_age_s': (
                    localization_test_readiness_age
                ),
                'localization_test_start_allowed': bool(
                    self._localization_test_start_enabled
                    and localization_test_fresh
                    and localization_test_readiness.get('ready', False)
                    and snapshot['localization_test_state'] in INACTIVE_STATES
                    and self._localization_test_start_client.service_is_ready()
                    and snapshot['mission_state'] in {
                        'UNKNOWN', *INACTIVE_STATES,
                    }
                    and snapshot['validation_state'] in {
                        'UNKNOWN', *INACTIVE_STATES,
                    }
                ),
                'timestamp': time.time(),
            }
        )
        return snapshot

    def _start_rejection(self, request: dict) -> str:
        if request.get('confirmation') != START_CONFIRMATION:
            return 'start confirmation phrase is missing'
        status = self._status_snapshot()
        if not self._operator_start_enabled:
            return 'operator start is locked by deployment configuration'
        if not status['start_service_ready']:
            return 'mission start service is unavailable'
        if status['readiness_age_s'] is None or status['readiness_age_s'] > 2.0:
            return 'mission readiness is stale'
        if not status['readiness'].get('ready', False):
            failures = status['readiness'].get('failures', [])
            return 'preflight blocked: ' + '; '.join(failures)
        if status['mission_state'] not in INACTIVE_STATES:
            return f"mission is active ({status['mission_state']})"
        return ''

    def _validation_start_rejection(self, request: dict) -> str:
        if request.get('confirmation') != VALIDATION_CONFIRMATION:
            return 'validation confirmation phrase is missing'
        checklist = request.get('checklist')
        if not isinstance(checklist, list) or set(checklist) != VALIDATION_CHECKLIST:
            return 'the complete physical flight checklist is required'
        status = self._status_snapshot()
        if not self._validation_start_enabled:
            return 'flight validation is locked by deployment configuration'
        if not status['validation_start_service_ready']:
            return 'flight validation start service is unavailable'
        age = status['validation_readiness_age_s']
        if age is None or age > 2.0:
            return 'flight validation readiness is stale'
        readiness = status['validation_readiness']
        if not readiness.get('ready', False):
            return 'validation blocked: ' + '; '.join(
                readiness.get('failures', [])
            )
        if status['validation_state'] not in {'UNKNOWN', *INACTIVE_STATES}:
            return f"flight validation is active ({status['validation_state']})"
        if status['mission_state'] not in {'UNKNOWN', *INACTIVE_STATES}:
            return f"cleaning mission is active ({status['mission_state']})"
        return ''

    def _localization_test_start_rejection(self, request: dict) -> str:
        if request.get('confirmation') != LOCALIZATION_TEST_CONFIRMATION:
            return 'localization test confirmation phrase is missing'
        checklist = request.get('checklist')
        if (
            not isinstance(checklist, list)
            or set(checklist) != LOCALIZATION_TEST_CHECKLIST
        ):
            return 'the complete localization test checklist is required'
        status = self._status_snapshot()
        if not self._localization_test_start_enabled:
            return 'localization test is locked by deployment configuration'
        if not status['localization_test_start_service_ready']:
            return 'localization test start service is unavailable'
        age = status['localization_test_readiness_age_s']
        if age is None or age > 2.0:
            return 'localization test readiness is stale'
        readiness = status['localization_test_readiness']
        if not readiness.get('ready', False):
            return 'localization test blocked: ' + '; '.join(
                readiness.get('failures', [])
            )
        if status['localization_test_state'] not in INACTIVE_STATES:
            return (
                'localization test is active '
                f"({status['localization_test_state']})"
            )
        if status['mission_state'] not in {'UNKNOWN', *INACTIVE_STATES}:
            return f"cleaning mission is active ({status['mission_state']})"
        if status['validation_state'] not in {'UNKNOWN', *INACTIVE_STATES}:
            return f"flight validation is active ({status['validation_state']})"
        return ''

    def _process_commands(self) -> None:
        try:
            pending = self._commands.get_nowait()
        except queue.Empty:
            return
        clients = {
            'start': self._start_client,
            'abort': self._abort_client,
            'validation_start': self._validation_start_client,
            'validation_abort': self._validation_abort_client,
            'localization_test_start': self._localization_test_start_client,
            'localization_test_abort': self._localization_test_abort_client,
        }
        client = clients[pending.command]
        if not client.service_is_ready():
            pending.result = {
                'ok': False,
                'error': f'mission {pending.command} service is unavailable',
            }
            pending.done.set()
            return
        future = client.call_async(Trigger.Request())

        def complete(result_future) -> None:
            try:
                response = result_future.result()
                pending.result = {
                    'ok': bool(response.success),
                    'message': str(response.message),
                }
                if not response.success:
                    pending.result['error'] = str(response.message)
            except Exception as exc:  # pragma: no cover - ROS transport failure
                pending.result = {'ok': False, 'error': str(exc)}
            pending.done.set()

        future.add_done_callback(complete)

    def destroy_node(self) -> bool:
        """Stop the local socket before destroying ROS resources."""
        if hasattr(self, '_server'):
            self._server.shutdown()
            self._server.server_close()
        try:
            path = Path(self._socket_path)
            if path.is_socket():
                path.unlink()
        except OSError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """Run the local-only operator gateway node."""
    rclpy.init(args=args)
    node = OperatorGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
