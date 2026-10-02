"""Airborne spray-reaction test: hover, spray once with feedforward, land."""

import math
import time
from typing import Optional

from da_daka_control.panel_mapping import quaternion_tilt_rad
from da_daka_control.spray_hover_test_fsm import (
    horizontal_hold_speed,
    SprayHoverState,
    SprayHoverTestFsm,
    StableWindow,
    vertical_hold_speed,
)
from da_daka_control.spray_reaction import apply_vertical_feedforward
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger


class SprayHoverTestNode(Node):
    """Run one hover-spray-settle-land cycle under explicit safety bounds."""

    def __init__(self) -> None:
        super().__init__('spray_hover_test')
        self._declare_parameters()
        self._load_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._fsm = SprayHoverTestFsm()

        self._state: Optional[State] = None
        self._pose: Optional[PoseStamped] = None
        self._pose_time_s: Optional[float] = None
        self._velocity: Optional[TwistStamped] = None
        self._velocity_time_s: Optional[float] = None
        self._battery_fraction: Optional[float] = None
        self._spray_ff_mps = 0.0
        self._spray_ff_time_s: Optional[float] = None
        self._spray_active = False

        self._launch_xyz: Optional[tuple[float, float, float]] = None
        self._prestream_started_s: Optional[float] = None
        self._offboard_requested_s: Optional[float] = None
        self._arm_requested_s: Optional[float] = None
        self._state_entered_s = time.monotonic()
        self._spray_started_s: Optional[float] = None
        self._spray_path_enabled = False
        self._land_requested_s: Optional[float] = None
        self._peak_height_error_m = 0.0
        self._peak_descent_mps = 0.0

        self._takeoff_window = StableWindow(self._takeoff_stable_duration_s)
        self._hover_window = StableWindow(self._hover_stable_duration_s)
        self._settle_window = StableWindow(self._settle_stable_duration_s)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(
            TwistStamped, self._command_topic, 10
        )
        self._state_publisher = self.create_publisher(
            String, self._state_topic, latched_qos
        )
        self._result_publisher = self.create_publisher(
            String, self._result_topic, latched_qos
        )

        self.create_subscription(
            State, '/mavros/state', self._state_callback, 10
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            10,
        )
        self.create_subscription(
            BatteryState, '/mavros/battery', self._battery_callback, 10
        )
        self.create_subscription(
            Float32, self._spray_ff_topic, self._spray_ff_callback, 10
        )
        self.create_subscription(
            Bool, '/spray/active', self._spray_active_callback, 10
        )

        self._arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming',
            callback_group=self._callback_group,
        )
        self._set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode',
            callback_group=self._callback_group,
        )
        self._land_client = self.create_client(
            CommandTOL, '/mavros/cmd/land',
            callback_group=self._callback_group,
        )
        self._spray_enable_client = self.create_client(
            SetBool, '/spray/enable', callback_group=self._callback_group,
        )
        self._spray_trigger_client = self.create_client(
            Trigger, '/spray/trigger', callback_group=self._callback_group,
        )
        self._spray_stop_client = self.create_client(
            Trigger, '/spray/stop', callback_group=self._callback_group,
        )
        self._reaction_enable_client = self.create_client(
            SetBool, self._reaction_enable_service,
            callback_group=self._callback_group,
        )

        self.create_service(
            Trigger, self._start_service, self._start,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger, self._abort_service, self._abort,
            callback_group=self._callback_group,
        )

        self.create_timer(1.0 / self._control_rate_hz, self._tick)
        self._publish_state()
        self.get_logger().info(
            'Spray hover test ready; '
            f'height={self._hover_height_m:.2f} m, '
            f'pulse={self._spray_pulse_s:.2f} s, '
            f'feedforward={"ON" if self._use_feedforward else "OFF"}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('hover_height_m', 3.0)
        self.declare_parameter('height_tolerance_m', 0.20)
        self.declare_parameter('position_tolerance_m', 0.50)
        self.declare_parameter('vertical_gain_per_s', 0.8)
        self.declare_parameter('horizontal_gain_per_s', 0.8)
        self.declare_parameter('max_vertical_speed_mps', 0.6)
        self.declare_parameter('max_horizontal_speed_mps', 0.5)
        self.declare_parameter('stable_max_speed_mps', 0.15)
        self.declare_parameter('takeoff_stable_duration_s', 2.0)
        self.declare_parameter('hover_stable_duration_s', 5.0)
        self.declare_parameter('settle_stable_duration_s', 5.0)
        self.declare_parameter('takeoff_timeout_s', 40.0)
        self.declare_parameter('hover_timeout_s', 40.0)
        self.declare_parameter('settle_timeout_s', 30.0)
        self.declare_parameter('disarm_timeout_s', 60.0)
        self.declare_parameter('spray_pulse_s', 3.0)
        self.declare_parameter('prestream_duration_s', 1.5)
        self.declare_parameter('command_timeout_s', 5.0)
        self.declare_parameter('input_timeout_s', 0.5)
        self.declare_parameter('spray_ff_timeout_s', 0.5)
        self.declare_parameter('minimum_battery_fraction', 0.30)
        self.declare_parameter('maximum_tilt_deg', 12.0)
        self.declare_parameter('maximum_height_error_m', 1.0)
        self.declare_parameter('use_feedforward', True)
        self.declare_parameter('require_spray_output', False)
        self.declare_parameter(
            'command_topic', '/mavros/setpoint_velocity/cmd_vel'
        )
        self.declare_parameter(
            'spray_ff_topic', '/spray_reaction/vertical_velocity_ff'
        )
        self.declare_parameter(
            'reaction_enable_service', '/spray_reaction/enable'
        )
        self.declare_parameter('state_topic', '/spray_hover_test/state')
        self.declare_parameter('result_topic', '/spray_hover_test/result')
        self.declare_parameter('start_service', '/spray_hover_test/start')
        self.declare_parameter('abort_service', '/spray_hover_test/abort')

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._control_rate_hz = float(value('control_rate_hz'))
        self._hover_height_m = float(value('hover_height_m'))
        self._height_tolerance_m = float(value('height_tolerance_m'))
        self._position_tolerance_m = float(value('position_tolerance_m'))
        self._vertical_gain_per_s = float(value('vertical_gain_per_s'))
        self._horizontal_gain_per_s = float(value('horizontal_gain_per_s'))
        self._max_vertical_speed_mps = float(value('max_vertical_speed_mps'))
        self._max_horizontal_speed_mps = float(
            value('max_horizontal_speed_mps')
        )
        self._stable_max_speed_mps = float(value('stable_max_speed_mps'))
        self._takeoff_stable_duration_s = float(
            value('takeoff_stable_duration_s')
        )
        self._hover_stable_duration_s = float(value('hover_stable_duration_s'))
        self._settle_stable_duration_s = float(
            value('settle_stable_duration_s')
        )
        self._takeoff_timeout_s = float(value('takeoff_timeout_s'))
        self._hover_timeout_s = float(value('hover_timeout_s'))
        self._settle_timeout_s = float(value('settle_timeout_s'))
        self._disarm_timeout_s = float(value('disarm_timeout_s'))
        self._spray_pulse_s = float(value('spray_pulse_s'))
        self._prestream_duration_s = float(value('prestream_duration_s'))
        self._command_timeout_s = float(value('command_timeout_s'))
        self._input_timeout_s = float(value('input_timeout_s'))
        self._spray_ff_timeout_s = float(value('spray_ff_timeout_s'))
        self._minimum_battery_fraction = float(
            value('minimum_battery_fraction')
        )
        self._maximum_tilt_rad = math.radians(float(value('maximum_tilt_deg')))
        self._maximum_height_error_m = float(value('maximum_height_error_m'))
        self._use_feedforward = bool(value('use_feedforward'))
        self._require_spray_output = bool(value('require_spray_output'))
        self._command_topic = str(value('command_topic'))
        self._spray_ff_topic = str(value('spray_ff_topic'))
        self._reaction_enable_service = str(value('reaction_enable_service'))
        self._state_topic = str(value('state_topic'))
        self._result_topic = str(value('result_topic'))
        self._start_service = str(value('start_service'))
        self._abort_service = str(value('abort_service'))

        positive = (
            self._control_rate_hz,
            self._hover_height_m,
            self._height_tolerance_m,
            self._position_tolerance_m,
            self._max_vertical_speed_mps,
            self._max_horizontal_speed_mps,
            self._stable_max_speed_mps,
            self._spray_pulse_s,
            self._input_timeout_s,
            self._spray_ff_timeout_s,
            self._maximum_height_error_m,
        )
        if not all(math.isfinite(item) and item > 0.0 for item in positive):
            raise ValueError('spray hover test positive values are invalid')
        if not 0.0 <= self._minimum_battery_fraction <= 1.0:
            raise ValueError('minimum_battery_fraction must be within [0, 1]')
        if not 0.0 < self._maximum_tilt_rad <= math.pi / 2.0:
            raise ValueError('maximum_tilt_deg must be within (0, 90]')
        if self._height_tolerance_m >= self._maximum_height_error_m:
            raise ValueError(
                'maximum_height_error_m must exceed height_tolerance_m'
            )

    def _state_callback(self, message: State) -> None:
        self._state = message

    def _pose_callback(self, message: PoseStamped) -> None:
        self._pose = message
        self._pose_time_s = time.monotonic()

    def _velocity_callback(self, message: TwistStamped) -> None:
        self._velocity = message
        self._velocity_time_s = time.monotonic()

    def _battery_callback(self, message: BatteryState) -> None:
        fraction = float(message.percentage)
        if math.isfinite(fraction) and 0.0 <= fraction <= 1.0:
            self._battery_fraction = fraction

    def _spray_ff_callback(self, message: Float32) -> None:
        speed_mps = float(message.data)
        if not math.isfinite(speed_mps):
            return
        self._spray_ff_mps = speed_mps
        self._spray_ff_time_s = time.monotonic()

    def _spray_active_callback(self, message: Bool) -> None:
        self._spray_active = bool(message.data)

    def _start(self, request, response):
        del request
        if self._fsm.active:
            response.success = False
            response.message = f'test already running ({self._fsm.state.name})'
            return response
        self._fsm.state = SprayHoverState.IDLE
        self._fsm.start()
        self._enter_state()
        response.success = True
        response.message = 'spray hover test requested'
        self.get_logger().warn('Spray hover test START requested')
        return response

    def _abort(self, request, response):
        del request
        if not self._fsm.active:
            response.success = False
            response.message = 'no active test'
            return response
        self._fail('operator abort requested', land=self._fsm.airborne)
        response.success = True
        response.message = 'aborting'
        return response

    def _enter_state(self) -> None:
        self._state_entered_s = time.monotonic()
        self._publish_state()

    def _publish_state(self) -> None:
        message = String()
        message.data = f'{self._fsm.state.name}:{self._fsm.reason}'
        self._state_publisher.publish(message)

    def _publish_result(self, status: str, detail: str) -> None:
        message = String()
        message.data = (
            f'{{"status":"{status}","detail":"{detail}",'
            f'"spray_fired":{str(self._fsm.spray_fired).lower()},'
            f'"peak_height_error_m":{self._peak_height_error_m:.3f},'
            f'"peak_descent_mps":{self._peak_descent_mps:.3f}}}'
        )
        self._result_publisher.publish(message)

    def _call_async(self, client, request) -> None:
        """Fire one service call without blocking the control loop."""
        if not client.service_is_ready():
            self.get_logger().warn(
                f'service {client.srv_name} is not ready; skipping call'
            )
            return
        client.call_async(request)

    def _set_spray_path(self, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._call_async(self._spray_enable_client, request)
        if self._use_feedforward:
            reaction_request = SetBool.Request()
            reaction_request.data = enabled
            self._call_async(self._reaction_enable_client, reaction_request)
        self._spray_path_enabled = enabled

    def _stop_spray(self) -> None:
        self._call_async(self._spray_stop_client, Trigger.Request())
        if self._spray_path_enabled:
            self._set_spray_path(False)

    def _request_land(self) -> None:
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = float('nan')
        request.longitude = float('nan')
        request.altitude = float('nan')
        self._call_async(self._land_client, request)
        self._land_requested_s = time.monotonic()

    def _fail(self, reason: str, land: bool) -> None:
        self.get_logger().error(f'Spray hover test abort: {reason}')
        self._stop_spray()
        self._publish_zero_command()
        self._fsm.abort(reason)
        self._enter_state()
        self._publish_result('ABORT', reason)
        if land:
            self._request_land()

    def _inputs_fresh(self, now_s: float) -> bool:
        for stamp in (self._pose_time_s, self._velocity_time_s):
            if stamp is None or now_s - stamp > self._input_timeout_s:
                return False
        return self._state is not None

    def _height_m(self) -> Optional[float]:
        if self._pose is None or self._launch_xyz is None:
            return None
        return float(self._pose.pose.position.z) - self._launch_xyz[2]

    def _horizontal_error_m(self) -> Optional[tuple[float, float]]:
        if self._pose is None or self._launch_xyz is None:
            return None
        return (
            self._launch_xyz[0] - float(self._pose.pose.position.x),
            self._launch_xyz[1] - float(self._pose.pose.position.y),
        )

    def _speed_mps(self) -> float:
        if self._velocity is None:
            return float('inf')
        linear = self._velocity.twist.linear
        return math.sqrt(
            float(linear.x) ** 2
            + float(linear.y) ** 2
            + float(linear.z) ** 2
        )

    def _tilt_rad(self) -> float:
        if self._pose is None:
            return 0.0
        orientation = self._pose.pose.orientation
        try:
            return quaternion_tilt_rad((
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ))
        except ValueError:
            # An un-initialised attitude must not kill the control timer.
            return 0.0

    def _effective_ff_mps(self, now_s: float) -> float:
        """Return the spray feedforward, or zero when it is stale/disabled."""
        if not self._use_feedforward:
            return 0.0
        if (
            self._spray_ff_time_s is None
            or now_s - self._spray_ff_time_s > self._spray_ff_timeout_s
        ):
            return 0.0
        return self._spray_ff_mps

    def _publish_zero_command(self) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        self._command_publisher.publish(command)

    def _publish_hold_command(self, target_height_m: float, now_s: float):
        """Hold the launch point horizontally and one height vertically."""
        height_m = self._height_m()
        horizontal = self._horizontal_error_m()
        if height_m is None or horizontal is None:
            self._publish_zero_command()
            return
        height_error_m = target_height_m - height_m
        base_climb_mps = vertical_hold_speed(
            height_error_m,
            self._vertical_gain_per_s,
            self._max_vertical_speed_mps,
        )
        climb_mps = apply_vertical_feedforward(
            base_climb_mps,
            self._effective_ff_mps(now_s),
            self._max_vertical_speed_mps,
        )
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        command.twist.linear.x = horizontal_hold_speed(
            horizontal[0],
            self._horizontal_gain_per_s,
            self._max_horizontal_speed_mps,
        )
        command.twist.linear.y = horizontal_hold_speed(
            horizontal[1],
            self._horizontal_gain_per_s,
            self._max_horizontal_speed_mps,
        )
        command.twist.linear.z = climb_mps
        self._command_publisher.publish(command)

    def _record_excursion(self, target_height_m: float) -> None:
        height_m = self._height_m()
        if height_m is not None:
            error_m = abs(target_height_m - height_m)
            self._peak_height_error_m = max(
                self._peak_height_error_m, error_m
            )
        if self._velocity is not None:
            descent_mps = -float(self._velocity.twist.linear.z)
            self._peak_descent_mps = max(self._peak_descent_mps, descent_mps)

    def _hold_is_stable(self, target_height_m: float) -> bool:
        height_m = self._height_m()
        horizontal = self._horizontal_error_m()
        if height_m is None or horizontal is None:
            return False
        return (
            abs(target_height_m - height_m) <= self._height_tolerance_m
            and math.hypot(horizontal[0], horizontal[1])
            <= self._position_tolerance_m
            and self._speed_mps() <= self._stable_max_speed_mps
        )

    def _airborne_guards_ok(self, now_s: float) -> bool:
        """Check the bounds that must hold for the whole airborne phase."""
        if self._state is None or not self._state.armed:
            self._fail('vehicle disarmed while airborne', land=False)
            return False
        if self._state.mode != 'OFFBOARD':
            self._fail(
                f'OFFBOARD lost while airborne (mode={self._state.mode})',
                land=False,
            )
            return False
        if self._tilt_rad() > self._maximum_tilt_rad:
            self._fail('tilt exceeded the airborne limit', land=True)
            return False
        height_m = self._height_m()
        if height_m is None:
            self._fail('height reference lost while airborne', land=True)
            return False
        if abs(self._hover_height_m - height_m) > self._maximum_height_error_m:
            self._fail(
                f'height error exceeded {self._maximum_height_error_m:.2f} m',
                land=True,
            )
            return False
        if (
            self._battery_fraction is not None
            and self._battery_fraction < self._minimum_battery_fraction
        ):
            self._fail('battery fell below the airborne floor', land=True)
            return False
        del now_s
        return True

    def _tick(self) -> None:
        now_s = time.monotonic()
        if not self._fsm.active:
            return
        if not self._inputs_fresh(now_s):
            self._fail('pose/velocity/state telemetry stale', land=True)
            return

        handler = {
            SprayHoverState.PRECHECK: self._tick_precheck,
            SprayHoverState.ARMING: self._tick_arming,
            SprayHoverState.TAKEOFF: self._tick_takeoff,
            SprayHoverState.HOVER_STABILISE: self._tick_hover,
            SprayHoverState.SPRAY_ARM: self._tick_spray_arm,
            SprayHoverState.SPRAY: self._tick_spray,
            SprayHoverState.SETTLE: self._tick_settle,
            SprayHoverState.LAND: self._tick_land,
            SprayHoverState.WAIT_DISARM: self._tick_wait_disarm,
        }.get(self._fsm.state)
        if handler is not None:
            handler(now_s)

    def _tick_precheck(self, now_s: float) -> None:
        failures = []
        if self._state is None or not self._state.connected:
            failures.append('MAVROS is not connected')
        if self._state is not None and self._state.armed:
            failures.append('vehicle is already armed')
        if self._battery_fraction is None:
            failures.append('battery telemetry unavailable')
        elif self._battery_fraction < self._minimum_battery_fraction:
            failures.append(
                f'battery {self._battery_fraction * 100.0:.0f}% is below the '
                f'{self._minimum_battery_fraction * 100.0:.0f}% floor'
            )
        if self._require_spray_output and not self._spray_enable_client.\
                service_is_ready():
            failures.append('spray controller service unavailable')
        if failures:
            if now_s - self._state_entered_s > self._command_timeout_s:
                self._fail('; '.join(failures), land=False)
            return
        self._launch_xyz = (
            float(self._pose.pose.position.x),
            float(self._pose.pose.position.y),
            float(self._pose.pose.position.z),
        )
        self._peak_height_error_m = 0.0
        self._peak_descent_mps = 0.0
        self._prestream_started_s = now_s
        self._fsm.precheck_complete()
        self._enter_state()
        self.get_logger().info(
            'Precheck complete; launch reference '
            f'({self._launch_xyz[0]:.2f}, {self._launch_xyz[1]:.2f}, '
            f'{self._launch_xyz[2]:.2f})'
        )

    def _tick_arming(self, now_s: float) -> None:
        self._publish_zero_command()
        if now_s - self._prestream_started_s < self._prestream_duration_s:
            return
        if self._state.mode != 'OFFBOARD':
            if (
                self._offboard_requested_s is None
                or now_s - self._offboard_requested_s
                > self._command_timeout_s
            ):
                request = SetMode.Request()
                request.custom_mode = 'OFFBOARD'
                self._call_async(self._set_mode_client, request)
                self._offboard_requested_s = now_s
            return
        if not self._state.armed:
            if (
                self._arm_requested_s is None
                or now_s - self._arm_requested_s > self._command_timeout_s
            ):
                request = CommandBool.Request()
                request.value = True
                self._call_async(self._arming_client, request)
                self._arm_requested_s = now_s
            return
        self._takeoff_window.reset()
        self._fsm.armed()
        self._enter_state()
        self.get_logger().warn('ARMED; climbing to the hover height')

    def _tick_takeoff(self, now_s: float) -> None:
        if not self._airborne_guards_ok(now_s):
            return
        self._publish_hold_command(self._hover_height_m, now_s)
        if self._takeoff_window.update(
            self._hold_is_stable(self._hover_height_m), now_s
        ):
            self._hover_window.reset()
            self._fsm.takeoff_complete()
            self._enter_state()
            return
        if now_s - self._state_entered_s > self._takeoff_timeout_s:
            self._fail('takeoff did not settle before the timeout', land=True)

    def _tick_hover(self, now_s: float) -> None:
        if not self._airborne_guards_ok(now_s):
            return
        self._publish_hold_command(self._hover_height_m, now_s)
        if self._hover_window.update(
            self._hold_is_stable(self._hover_height_m), now_s
        ):
            self._set_spray_path(True)
            self._fsm.hover_stable()
            self._enter_state()
            self.get_logger().warn(
                'Hover settled; enabling valve and feedforward'
            )
            return
        if now_s - self._state_entered_s > self._hover_timeout_s:
            self._fail('hover did not settle before the timeout', land=True)

    def _tick_spray_arm(self, now_s: float) -> None:
        if not self._airborne_guards_ok(now_s):
            return
        self._publish_hold_command(self._hover_height_m, now_s)
        if now_s - self._state_entered_s < self._command_timeout_s:
            return
        self._call_async(self._spray_trigger_client, Trigger.Request())
        self._spray_started_s = now_s
        self._fsm.spray_path_ready()
        self._enter_state()
        self.get_logger().warn('SPRAY pulse commanded')

    def _tick_spray(self, now_s: float) -> None:
        if not self._airborne_guards_ok(now_s):
            return
        self._publish_hold_command(self._hover_height_m, now_s)
        self._record_excursion(self._hover_height_m)
        if now_s - self._spray_started_s >= self._spray_pulse_s:
            self._settle_window.reset()
            self._fsm.spray_complete()
            self._enter_state()

    def _tick_settle(self, now_s: float) -> None:
        if not self._airborne_guards_ok(now_s):
            return
        self._publish_hold_command(self._hover_height_m, now_s)
        self._record_excursion(self._hover_height_m)
        if self._settle_window.update(
            self._hold_is_stable(self._hover_height_m), now_s
        ):
            self._stop_spray()
            self._fsm.settled()
            self._enter_state()
            self.get_logger().warn(
                'Hover recovered after spray; '
                f'peak height error {self._peak_height_error_m:.3f} m, '
                f'peak descent {self._peak_descent_mps:.3f} m/s'
            )
            return
        if now_s - self._state_entered_s > self._settle_timeout_s:
            self._fail('hover did not recover after the spray', land=True)

    def _tick_land(self, now_s: float) -> None:
        self._stop_spray()
        self._request_land()
        self._fsm.landing_started()
        self._enter_state()
        del now_s

    def _tick_wait_disarm(self, now_s: float) -> None:
        if self._state is not None and not self._state.armed:
            self._fsm.disarmed()
            self._enter_state()
            self._publish_result('COMPLETE', 'landed and disarmed')
            self.get_logger().warn(
                'Spray hover test COMPLETE; '
                f'peak height error {self._peak_height_error_m:.3f} m, '
                f'peak descent {self._peak_descent_mps:.3f} m/s'
            )
            return
        if now_s - self._land_requested_s > self._disarm_timeout_s:
            self._fail('vehicle did not disarm after landing', land=False)

    def destroy_node(self) -> bool:
        """Close the spray path and stop commanding before shutdown."""
        self._stop_spray()
        self._publish_zero_command()
        return super().destroy_node()


def main(args=None) -> None:
    """Spin the airborne spray-reaction test node."""
    rclpy.init(args=args)
    node = SprayHoverTestNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
