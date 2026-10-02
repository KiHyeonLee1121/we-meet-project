"""
Publish a bounded vertical feedforward estimate for spray reaction.

The node never publishes a MAVROS setpoint. It consumes the spray controller's
configured-duration activity estimate and publishes an offset that the distance
controller may add only when its own independent ``spray_ff_enabled`` gate is
open. Both gates default closed until physical measurements are approved.
"""

import math
import time
from typing import Optional

from da_daka_control.spray_reaction import (
    nozzle_area_m2,
    RampShaper,
    reaction_force_n,
    solve_operating_point,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


class SprayReactionCompensatorNode(Node):
    """Convert an estimated spray-active window into feedforward velocity."""

    def __init__(self) -> None:
        super().__init__('spray_reaction_compensator')
        self._declare_parameters()
        self._load_parameters()

        operating_point = solve_operating_point(
            pump_open_flow_m3s=self._pump_open_flow_lpm / 60000.0,
            pump_shutoff_pa=self._pump_shutoff_bar * 1.0e5,
            nozzle_area_m2_=nozzle_area_m2(self._nozzle_diameter_m),
            discharge_coefficient=self._discharge_coefficient,
            water_density_kgm3=self._water_density_kgm3,
        )
        self._steady_force_n = reaction_force_n(
            operating_point.flow_m3s,
            operating_point.velocity_mps,
            self._water_density_kgm3,
        )
        self._shaper = RampShaper(self._ramp_time_s)
        self._spray_active = False
        self._last_spray_state_s: Optional[float] = None
        self._distance_control_enabled = False
        self._manual_enabled = self._enabled_on_startup
        self._last_tick_s = time.monotonic()

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ff_publisher = self.create_publisher(
            Float32, self._vertical_ff_topic, 10
        )
        self._force_publisher = self.create_publisher(
            Float32, self._force_estimate_topic, 10
        )
        self._enabled_publisher = self.create_publisher(
            Bool, self._enabled_state_topic, state_qos
        )
        self.create_subscription(
            Bool,
            self._spray_active_topic,
            self._spray_active_callback,
            10,
        )
        if self._follow_distance_control:
            self.create_subscription(
                Bool,
                self._distance_control_enabled_topic,
                self._distance_control_callback,
                state_qos,
            )
        self.create_service(
            SetBool, self._enable_service, self._enable_callback
        )
        self.create_timer(1.0 / self._control_rate_hz, self._tick)
        self._publish_enabled()
        self._publish(0.0, 0.0)
        self.get_logger().info(
            'Spray reaction feedforward ready; '
            f'output_enabled={self._output_enabled}; '
            f'estimated P={operating_point.pressure_pa / 1.0e5:.3f} bar, '
            f'Q={operating_point.flow_m3s * 60000.0:.2f} L/min, '
            f'F={self._steady_force_n:.3f} N'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('spray_active_topic', '/spray/active')
        self.declare_parameter(
            'vertical_ff_topic', '/spray_reaction/vertical_velocity_ff'
        )
        self.declare_parameter(
            'force_estimate_topic', '/spray_reaction/force_estimate_n'
        )
        self.declare_parameter('enable_service', '/spray_reaction/enable')
        self.declare_parameter(
            'enabled_state_topic', '/spray_reaction/enabled'
        )
        self.declare_parameter('output_enabled', False)
        self.declare_parameter('enabled_on_startup', False)
        self.declare_parameter('follow_distance_control', False)
        self.declare_parameter(
            'distance_control_enabled_topic', '/distance_control/enabled'
        )
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('nozzle_diameter_m', 0.006)
        self.declare_parameter('pump_open_flow_lpm', 5.6)
        self.declare_parameter('pump_shutoff_bar', 2.8)
        self.declare_parameter('discharge_coefficient', 0.7)
        self.declare_parameter('water_density_kgm3', 1000.0)
        self.declare_parameter('drone_mass_kg', 2.8)
        self.declare_parameter('ff_gain_s', 1.0)
        self.declare_parameter('ramp_time_s', 0.3)
        self.declare_parameter('max_ff_speed_mps', 0.3)
        self.declare_parameter('spray_state_timeout_s', 0.5)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._spray_active_topic = str(value('spray_active_topic'))
        self._vertical_ff_topic = str(value('vertical_ff_topic'))
        self._force_estimate_topic = str(value('force_estimate_topic'))
        self._enable_service = str(value('enable_service'))
        self._enabled_state_topic = str(value('enabled_state_topic'))
        self._output_enabled = bool(value('output_enabled'))
        self._enabled_on_startup = bool(value('enabled_on_startup'))
        self._follow_distance_control = bool(value('follow_distance_control'))
        self._distance_control_enabled_topic = str(
            value('distance_control_enabled_topic')
        )
        self._control_rate_hz = float(value('control_rate_hz'))
        self._nozzle_diameter_m = float(value('nozzle_diameter_m'))
        self._pump_open_flow_lpm = float(value('pump_open_flow_lpm'))
        self._pump_shutoff_bar = float(value('pump_shutoff_bar'))
        self._discharge_coefficient = float(value('discharge_coefficient'))
        self._water_density_kgm3 = float(value('water_density_kgm3'))
        self._drone_mass_kg = float(value('drone_mass_kg'))
        self._ff_gain_s = float(value('ff_gain_s'))
        self._ramp_time_s = float(value('ramp_time_s'))
        self._max_ff_speed_mps = float(value('max_ff_speed_mps'))
        self._spray_state_timeout_s = float(value('spray_state_timeout_s'))
        positive = (
            self._control_rate_hz,
            self._nozzle_diameter_m,
            self._pump_open_flow_lpm,
            self._pump_shutoff_bar,
            self._discharge_coefficient,
            self._water_density_kgm3,
            self._drone_mass_kg,
            self._max_ff_speed_mps,
            self._spray_state_timeout_s,
        )
        if not all(math.isfinite(item) and item > 0.0 for item in positive):
            raise ValueError('spray reaction positive parameters are invalid')
        if not math.isfinite(self._ff_gain_s) or self._ff_gain_s < 0.0:
            raise ValueError('ff_gain_s must be finite and non-negative')
        if not math.isfinite(self._ramp_time_s) or self._ramp_time_s < 0.0:
            raise ValueError('ramp_time_s must be finite and non-negative')

    def _effective_enabled(self) -> bool:
        requested = (
            self._distance_control_enabled
            if self._follow_distance_control
            else self._manual_enabled
        )
        return self._output_enabled and requested

    def _spray_active_callback(self, message: Bool) -> None:
        self._spray_active = bool(message.data)
        self._last_spray_state_s = time.monotonic()

    def _distance_control_callback(self, message: Bool) -> None:
        was_enabled = self._effective_enabled()
        self._distance_control_enabled = bool(message.data)
        self._handle_enable_change(was_enabled)

    def _enable_callback(self, request, response):
        if self._follow_distance_control:
            response.success = False
            response.message = (
                'manual enable blocked while following distance control'
            )
            return response
        if request.data and not self._output_enabled:
            response.success = False
            response.message = 'output_enabled=false blocks feedforward'
            return response
        was_enabled = self._effective_enabled()
        self._manual_enabled = bool(request.data)
        self._handle_enable_change(was_enabled)
        response.success = True
        response.message = (
            'enabled' if self._effective_enabled() else 'disabled'
        )
        return response

    def _handle_enable_change(self, was_enabled: bool) -> None:
        enabled = self._effective_enabled()
        if enabled == was_enabled:
            return
        if not enabled:
            self._shaper.reset()
            self._publish(0.0, 0.0)
        self._publish_enabled()

    def _publish_enabled(self) -> None:
        self._enabled_publisher.publish(
            Bool(data=self._effective_enabled())
        )

    def _publish(self, velocity_ff_mps: float, force_n: float) -> None:
        self._ff_publisher.publish(Float32(data=velocity_ff_mps))
        self._force_publisher.publish(Float32(data=force_n))

    def _tick(self) -> None:
        now_s = time.monotonic()
        dt_s = max(1.0e-3, min(0.2, now_s - self._last_tick_s))
        self._last_tick_s = now_s
        if not self._effective_enabled():
            self._shaper.reset()
            self._publish(0.0, 0.0)
            return
        spray_state_fresh = (
            self._last_spray_state_s is not None
            and now_s - self._last_spray_state_s
            <= self._spray_state_timeout_s
        )
        level = self._shaper.update(
            self._spray_active and spray_state_fresh,
            dt_s,
        )
        force_n = self._steady_force_n * level
        velocity_ff_mps = -self._ff_gain_s * force_n / self._drone_mass_kg
        velocity_ff_mps = max(
            -self._max_ff_speed_mps,
            min(self._max_ff_speed_mps, velocity_ff_mps),
        )
        self._publish(velocity_ff_mps, force_n)

    def destroy_node(self) -> bool:
        """Publish zero before releasing ROS resources."""
        self._shaper.reset()
        self._publish(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the spray reaction compensator node."""
    rclpy.init(args=args)
    node = SprayReactionCompensatorNode()
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
