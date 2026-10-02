"""Ground-only, fail-closed solenoid pulse service via PX4 camera trigger."""

import threading
import time

from da_daka_control.solenoid_bench_core import (
    bench_interlock_failures,
    BenchSnapshot,
    digicam_command_parameters,
    MAV_CMD_DO_DIGICAM_CONTROL,
    MAV_RESULT_ACCEPTED,
)
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandLong
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SolenoidBenchNode(Node):
    """Expose one fixed-duration PX4 GPIO pulse with ground safety gates."""

    def __init__(self) -> None:
        super().__init__('solenoid_bench')
        self.declare_parameter('bench_test_approved', False)
        self.declare_parameter('telemetry_timeout_s', 1.0)
        self.declare_parameter('cooldown_s', 5.0)
        self.declare_parameter('max_pulses_per_session', 3)
        self.declare_parameter('command_timeout_s', 3.0)
        self.declare_parameter('expected_trigger_duration_ms', 3000)
        self.declare_parameter('service_name', '/spray/bench_pulse')
        self.declare_parameter('command_service', '/mavros/cmd/command')

        self._approved = bool(
            self.get_parameter('bench_test_approved').value
        )
        self._telemetry_timeout_s = self._positive_float(
            'telemetry_timeout_s'
        )
        self._cooldown_s = self._nonnegative_float('cooldown_s')
        self._max_pulses = self._positive_int('max_pulses_per_session')
        self._command_timeout_s = self._positive_float(
            'command_timeout_s'
        )
        self._duration_ms = self._positive_int(
            'expected_trigger_duration_ms'
        )
        service_name = str(self.get_parameter('service_name').value)
        command_service = str(self.get_parameter('command_service').value)

        self._lock = threading.Lock()
        self._connected = None
        self._armed = None
        self._landed_state = None
        self._state_time_s = None
        self._extended_state_time_s = None
        self._command_pending = False
        self._attempts = 0
        self._last_attempt_time_s = None

        self._callback_group = ReentrantCallbackGroup()
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_publisher = self.create_publisher(
            String,
            '/spray/bench_state',
            status_qos,
        )
        self._result_publisher = self.create_publisher(
            String,
            '/spray/bench_result',
            status_qos,
        )
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self._extended_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._command_client = self.create_client(
            CommandLong,
            command_service,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            service_name,
            self._pulse_callback,
            callback_group=self._callback_group,
        )
        self.create_timer(
            1.0,
            self._publish_state,
            callback_group=self._callback_group,
        )
        self._publish_state()
        self.get_logger().warning(
            'Ground bench ready: approved=%s, expected_duration=%dms, '
            'max_attempts=%d. ACK does not prove valve movement.'
            % (self._approved, self._duration_ms, self._max_pulses)
        )

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _nonnegative_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must not be negative')
        return value

    def _positive_int(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _state_callback(self, message: State) -> None:
        now_s = time.monotonic()
        with self._lock:
            self._connected = bool(message.connected)
            self._armed = bool(message.armed)
            self._state_time_s = now_s

    def _extended_state_callback(self, message: ExtendedState) -> None:
        with self._lock:
            self._landed_state = int(message.landed_state)
            self._extended_state_time_s = time.monotonic()

    def _snapshot(self, now_s: float) -> BenchSnapshot:
        return BenchSnapshot(
            now_s=now_s,
            configuration_approved=self._approved,
            connected=self._connected,
            armed=self._armed,
            landed_state=self._landed_state,
            state_time_s=self._state_time_s,
            extended_state_time_s=self._extended_state_time_s,
            command_pending=self._command_pending,
            attempts=self._attempts,
            last_attempt_time_s=self._last_attempt_time_s,
        )

    def _pulse_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        now_s = time.monotonic()
        with self._lock:
            failures = bench_interlock_failures(
                self._snapshot(now_s),
                self._telemetry_timeout_s,
                self._cooldown_s,
                self._max_pulses,
            )
            if failures:
                return self._reject(response, '; '.join(failures))
            if not self._command_client.service_is_ready():
                return self._reject(
                    response,
                    'MAVROS command service is unavailable',
                )
            self._command_pending = True
            self._attempts += 1
            self._last_attempt_time_s = now_s

        command_request = CommandLong.Request()
        command_request.broadcast = False
        command_request.command = MAV_CMD_DO_DIGICAM_CONTROL
        command_request.confirmation = 0
        parameters = digicam_command_parameters()
        for index, value in enumerate(parameters, start=1):
            setattr(command_request, f'param{index}', value)

        completed = threading.Event()
        command_future = self._command_client.call_async(command_request)
        command_future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(self._command_timeout_s):
            with self._lock:
                self._command_pending = False
            message = (
                'MAVROS command ACK timed out; output state is unknown and '
                'this attempt remains counted'
            )
            self._publish_result(message)
            response.success = False
            response.message = message
            return response

        with self._lock:
            self._command_pending = False
        try:
            command_response = command_future.result()
        except Exception as error:
            message = f'MAVROS command failed: {error}'
            self._publish_result(message)
            response.success = False
            response.message = message
            return response

        accepted = bool(
            command_response.success
            and command_response.result == MAV_RESULT_ACCEPTED
        )
        if accepted:
            message = (
                f'PX4 accepted one trigger (expected {self._duration_ms}ms); '
                'verify TRIG_ACT_TIME and physical valve movement separately'
            )
        else:
            message = (
                'PX4 rejected trigger: '
                f'success={command_response.success} '
                f'MAV_RESULT={command_response.result}'
            )
        self._publish_result(message)
        response.success = accepted
        response.message = message
        return response

    def _reject(
        self,
        response: Trigger.Response,
        message: str,
    ) -> Trigger.Response:
        full_message = f'BENCH_PULSE_BLOCKED: {message}'
        self._publish_result(full_message)
        response.success = False
        response.message = full_message
        return response

    def _publish_state(self) -> None:
        now_s = time.monotonic()
        with self._lock:
            snapshot = self._snapshot(now_s)
            failures = bench_interlock_failures(
                snapshot,
                self._telemetry_timeout_s,
                self._cooldown_s,
                self._max_pulses,
            )
        state = 'READY' if not failures else 'LOCKED'
        details = 'none' if not failures else '; '.join(failures)
        self._state_publisher.publish(
            String(
                data=(
                    f'{state} attempts={snapshot.attempts}/'
                    f'{self._max_pulses} blockers={details}'
                )
            )
        )

    def _publish_result(self, value: str) -> None:
        self._result_publisher.publish(String(data=value))
        if value.startswith('PX4 accepted'):
            self.get_logger().info(value)
        else:
            self.get_logger().warning(value)


def main(args=None) -> None:
    """Run the bench node with enough threads to receive command ACKs."""
    rclpy.init(args=args)
    node = SolenoidBenchNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
