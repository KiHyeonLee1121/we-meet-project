"""ROS 2 services for Pixhawk AUX5 camera-trigger spray control."""

import json
import math
import time

from da_daka_control.spray_actuator import (
    pixhawk_disable_trigger_fields,
    pixhawk_one_shot_fields,
    SprayPulseGate,
    SprayResult,
)
from mavros_msgs.srv import CommandLong
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class SprayControllerNode(Node):
    """Translate mission spray services into PX4 camera-trigger commands."""

    def __init__(self) -> None:
        super().__init__('spray_controller')
        self._declare_parameters()
        backend = str(self.get_parameter('backend').value).lower()
        output_enabled = bool(self.get_parameter('output_enabled').value)
        pulse_duration_s = float(
            self.get_parameter('pulse_duration_s').value
        )
        maximum_pulse_s = float(
            self.get_parameter('maximum_pulse_s').value
        )
        if not math.isclose(pulse_duration_s, 3.0, abs_tol=1e-9):
            raise ValueError('pulse_duration_s must be exactly 3.0 seconds')
        if maximum_pulse_s < 3.0:
            raise ValueError('maximum_pulse_s must be at least 3.0 seconds')
        self._gate = SprayPulseGate(
            backend=backend,
            output_enabled=output_enabled,
            pulse_duration_s=pulse_duration_s,
            minimum_pulse_s=float(
                self.get_parameter('minimum_pulse_s').value
            ),
            maximum_pulse_s=maximum_pulse_s,
            cooldown_s=float(self.get_parameter('cooldown_s').value),
        )
        self._backend = backend
        self._command_group = ReentrantCallbackGroup()
        self._mavros_command_client = self.create_client(
            CommandLong,
            str(self.get_parameter('mavros_command_service').value),
            callback_group=self._command_group,
        )
        status_rate_hz = float(self.get_parameter('status_rate_hz').value)
        if status_rate_hz <= 0.0:
            raise ValueError('status_rate_hz must be positive')
        self._active_publisher = self.create_publisher(
            Bool, '/spray/active', 10
        )
        self._status_publisher = self.create_publisher(
            String, '/spray/status', 10
        )
        self.create_service(
            SetBool,
            '/spray/enable',
            self._enable,
            callback_group=self._command_group,
        )
        self.create_service(
            Trigger,
            '/spray/trigger',
            self._trigger,
            callback_group=self._command_group,
        )
        self.create_service(
            Trigger,
            '/spray/stop',
            self._stop,
            callback_group=self._command_group,
        )
        self._timer = self.create_timer(
            1.0 / status_rate_hz,
            self._publish_status,
        )
        mode = 'LIVE' if output_enabled else 'BLOCKED'
        self.get_logger().info(
            f'Spray controller backend={backend}, output={mode}; '
            'live path is MAVROS -> Pixhawk Camera_Trigger -> AUX5'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('output_enabled', False)
        self.declare_parameter(
            'mavros_command_service', '/mavros/cmd/command'
        )
        self.declare_parameter('pulse_duration_s', 3.0)
        self.declare_parameter('minimum_pulse_s', 0.05)
        self.declare_parameter('maximum_pulse_s', 3.0)
        self.declare_parameter('cooldown_s', 2.0)
        self.declare_parameter('status_rate_hz', 10.0)

    def _enable(self, request, response):
        result = self._gate.set_enabled(bool(request.data))
        response.success = result.success
        response.message = result.message
        self._publish_status()
        return response

    async def _trigger(self, _request, response):
        started = self._gate.begin_trigger()
        if not started.success:
            response.success = False
            response.message = started.message
            return response
        if self._backend == 'mock':
            result = self._gate.finish_trigger(True)
        else:
            command_result = await self._send_pixhawk_command(
                pixhawk_one_shot_fields()
            )
            result = self._gate.finish_trigger(command_result.success)
            if not command_result.success:
                result = command_result
        response.success = result.success
        response.message = result.message
        self._publish_status()
        return response

    async def _stop(self, _request, response):
        if self._backend == 'mock':
            result = self._gate.stop(can_cancel_active=True)
        else:
            command_result = await self._send_pixhawk_command(
                pixhawk_disable_trigger_fields()
            )
            if command_result.success:
                result = self._gate.stop(can_cancel_active=False)
            else:
                result = command_result
        response.success = result.success
        response.message = result.message
        self._publish_status()
        return response

    async def _send_pixhawk_command(self, fields) -> SprayResult:
        if not self._mavros_command_client.service_is_ready():
            self._gate.cancel_pending()
            return SprayResult(False, 'MAVROS command service is not ready')
        request = CommandLong.Request()
        for name, value in fields.items():
            setattr(request, name, value)
        try:
            response = await self._mavros_command_client.call_async(request)
        except Exception as exc:
            self._gate.cancel_pending()
            return SprayResult(False, f'MAVROS command call failed: {exc}')
        if not response.success:
            return SprayResult(
                False,
                f'Pixhawk command rejected (MAV_RESULT={response.result})',
            )
        return SprayResult(
            True,
            f'Pixhawk command accepted (MAV_RESULT={response.result})',
        )

    def _publish_status(self) -> None:
        status = self._gate.status(time.monotonic())
        self._active_publisher.publish(Bool(data=status.active))
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        'output_enabled': status.output_enabled,
                        'session_enabled': status.session_enabled,
                        'active': status.active,
                        'active_source': 'configured_duration_estimate',
                        'request_pending': status.request_pending,
                        'pulse_count': status.pulse_count,
                        'backend': status.backend,
                    },
                    separators=(',', ':'),
                )
            )
        )


def main(args=None) -> None:
    """Run the spray controller node."""
    rclpy.init(args=args)
    node = SprayControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
