"""Coordinate the DA-DAKA distance-control flight as a non-blocking FSM."""

import csv
from datetime import datetime
from enum import auto, Enum
import json
import math
from pathlib import Path
import time
from typing import Callable, Dict, IO, Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import Altitude, ExtendedState, State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger


class MissionState(Enum):
    """Mission states published by name on /mission/state."""

    IDLE = auto()
    PRECHECK = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAIT_HOVER = auto()
    CHECK_SENSOR = auto()
    PRESTREAM_SETPOINT = auto()
    ENABLE_DISTANCE_CONTROL = auto()
    ENTER_OFFBOARD = auto()
    DISTANCE_CONTROL = auto()
    TARGET_HOLD = auto()
    LOITER_HANDOVER = auto()
    AUTO_LAND = auto()
    WAIT_DISARM = auto()
    COMPLETE = auto()
    ABORT = auto()


class StableWindow:
    """Require a condition to remain true continuously for a duration."""

    def __init__(self, duration_s: float) -> None:
        if duration_s <= 0.0:
            raise ValueError('duration_s must be greater than zero')
        self.duration_s = duration_s
        self._since: Optional[float] = None

    def reset(self) -> None:
        """Reset the continuous-condition timer."""
        self._since = None

    def update(self, condition: bool, now_s: float) -> bool:
        """Return true once condition has stayed true for duration_s."""
        if not condition:
            self.reset()
            return False
        if self._since is None:
            self._since = now_s
        return now_s - self._since >= self.duration_s


def sag_compensated_remaining(
    *,
    voltage: Optional[float],
    current: Optional[float],
    cell_count: int,
    internal_resistance_ohm: float,
    empty_volts_per_cell: float,
    full_volts_per_cell: float,
) -> Optional[float]:
    """Return open-circuit state of charge, or None when unusable."""
    # PX4 derives its percentage from the sagging terminal voltage, so a 22 A
    # hover reads far emptier than the pack really is. The 2026-08-23 flights
    # logged 31% at rest collapsing to 4% under load and recovering to 20% on
    # the ground, which aborted two otherwise healthy missions. Add the
    # resistive drop back before mapping voltage to charge.
    if cell_count <= 0:
        raise ValueError('battery cell count must be positive')
    if internal_resistance_ohm < 0.0:
        raise ValueError('battery internal resistance must not be negative')
    if not math.isfinite(internal_resistance_ohm):
        raise ValueError('battery internal resistance must be finite')
    if not empty_volts_per_cell < full_volts_per_cell:
        raise ValueError('battery empty voltage must be below full voltage')
    if voltage is None or current is None:
        return None
    if not math.isfinite(voltage) or not math.isfinite(current):
        return None
    if voltage <= 0.0:
        return None
    # MAVROS reports discharge as a negative current.
    open_circuit_v = voltage + abs(current) * internal_resistance_ohm
    per_cell_v = open_circuit_v / cell_count
    ratio = (per_cell_v - empty_volts_per_cell) / (
        full_volts_per_cell - empty_volts_per_cell
    )
    return min(1.0, max(0.0, ratio))


def inflight_battery_failure(
    *,
    now_s: float,
    timeout_s: float,
    battery_remaining: Optional[float],
    battery_time_s: Optional[float],
    minimum_battery_remaining: float,
    below_since_s: Optional[float] = None,
    sustained_below_s: float = 0.0,
) -> Optional[str]:
    """Return a landing reason when fresh in-flight battery is below limit."""
    # below_since_s is when the reading first dropped under the limit. A pack
    # only aborts the mission once it has stayed low for sustained_below_s, so
    # a throttle transient cannot end a flight. PX4's own low-battery failsafe
    # stays independent and authoritative.
    if sustained_below_s < 0.0 or not math.isfinite(sustained_below_s):
        raise ValueError('sustained battery window must be finite and positive')
    if battery_time_s is None or now_s - battery_time_s > timeout_s:
        return None
    if battery_remaining is None or not math.isfinite(battery_remaining):
        return None
    if battery_remaining >= minimum_battery_remaining:
        return None
    if sustained_below_s > 0.0:
        if below_since_s is None:
            return None
        if now_s - below_since_s < sustained_below_s:
            return None
    return (
        f'in-flight battery {battery_remaining:.0%} below '
        f'{minimum_battery_remaining:.0%}'
    )


def status_failures(
    *,
    now_s: float,
    timeout_s: float,
    battery_remaining: Optional[float],
    battery_time_s: Optional[float],
    minimum_battery_remaining: float,
    landed_state: Optional[int],
    extended_state_time_s: Optional[float],
    require_on_ground: bool,
    sensors_enabled: Optional[int],
    sensors_health: Optional[int],
    sys_status_time_s: Optional[float],
    require_enabled_sensors_healthy: bool,
    ignored_unhealthy_sensor_mask: int,
) -> list[str]:
    """Return actionable PX4 status failures for mission gating."""
    failures = []
    if battery_time_s is None or now_s - battery_time_s > timeout_s:
        failures.append('battery telemetry unavailable or stale')
    elif battery_remaining is None:
        failures.append('battery remaining is invalid')
    elif battery_remaining < minimum_battery_remaining:
        failures.append(
            f'battery {battery_remaining:.0%} below '
            f'{minimum_battery_remaining:.0%}'
        )

    if extended_state_time_s is None or (
        now_s - extended_state_time_s > timeout_s
    ):
        failures.append('extended state unavailable or stale')
    elif require_on_ground and landed_state != 1:
        failures.append(f'vehicle is not confirmed on ground ({landed_state})')

    if require_enabled_sensors_healthy:
        if sys_status_time_s is None or now_s - sys_status_time_s > timeout_s:
            failures.append('PX4 system status unavailable or stale')
        elif sensors_enabled is None or sensors_health is None:
            failures.append('PX4 sensor health is unavailable')
        else:
            unhealthy_mask = (
                sensors_enabled
                & ~sensors_health
                & ~ignored_unhealthy_sensor_mask
            )
            if unhealthy_mask:
                failures.append(
                    f'PX4 enabled sensors unhealthy (mask=0x{unhealthy_mask:x})'
                )
    return failures


class MissionManagerNode(Node):
    """Run the distance-control mission without blocking ROS callbacks."""

    OFFBOARD_CONTROL_STATES = {
        MissionState.WAIT_HOVER,
        MissionState.DISTANCE_CONTROL,
        MissionState.TARGET_HOLD,
    }
    ACTIVE_STATES = {
        MissionState.PRECHECK,
        MissionState.ARMING,
        MissionState.TAKEOFF,
        MissionState.WAIT_HOVER,
        MissionState.CHECK_SENSOR,
        MissionState.PRESTREAM_SETPOINT,
        MissionState.ENABLE_DISTANCE_CONTROL,
        MissionState.ENTER_OFFBOARD,
        MissionState.DISTANCE_CONTROL,
        MissionState.TARGET_HOLD,
        MissionState.LOITER_HANDOVER,
        MissionState.AUTO_LAND,
        MissionState.WAIT_DISARM,
    }
    SENSOR_CRITICAL_STATES = {
        MissionState.ARMING,
        MissionState.TAKEOFF,
        MissionState.WAIT_HOVER,
        MissionState.CHECK_SENSOR,
        MissionState.PRESTREAM_SETPOINT,
        MissionState.ENABLE_DISTANCE_CONTROL,
        MissionState.ENTER_OFFBOARD,
        MissionState.DISTANCE_CONTROL,
        MissionState.TARGET_HOLD,
    }
    ARMED_REQUIRED_STATES = {
        MissionState.TAKEOFF,
        MissionState.WAIT_HOVER,
        MissionState.CHECK_SENSOR,
        MissionState.PRESTREAM_SETPOINT,
        MissionState.ENABLE_DISTANCE_CONTROL,
        MissionState.ENTER_OFFBOARD,
        MissionState.DISTANCE_CONTROL,
        MissionState.TARGET_HOLD,
        MissionState.LOITER_HANDOVER,
    }

    def __init__(self) -> None:
        super().__init__('mission_manager')
        self._declare_parameters()
        self._load_parameters()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String,
            '/mission/state',
            latched_qos,
        )
        self._result_publisher = self.create_publisher(
            String,
            '/mission/result',
            latched_qos,
        )
        self._readiness_publisher = self.create_publisher(
            String,
            '/mission/readiness',
            latched_qos,
        )
        self._start_service = self.create_service(
            Trigger,
            '/mission/start',
            self._start_callback,
        )
        self._abort_service = self.create_service(
            Trigger,
            '/mission/abort',
            self._abort_callback,
        )

        self.create_subscription(
            Range,
            '/distance/filtered',
            self._range_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            '/distance_control/enabled',
            self._enabled_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/distance_control/target_reached',
            self._target_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/enabled',
            self._local_takeoff_enabled_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/target_reached',
            self._local_takeoff_target_callback,
            latched_qos,
        )
        self.create_subscription(
            Float32,
            '/distance_control/error',
            self._control_error_callback,
            10,
        )
        self.create_subscription(
            Float32,
            '/distance_control/vertical_speed_cmd',
            self._control_command_callback,
            10,
        )
        self.create_subscription(
            State,
            '/mavros/state',
            self._mavros_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self._altitude_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self._extended_state_callback,
            10,
        )
        self.create_subscription(
            SysStatus,
            '/mavros/sys_status',
            self._sys_status_callback,
            10,
        )
        self.create_subscription(
            String,
            '/autonomous_cleaning/state',
            self._autonomous_state_callback,
            latched_qos,
        )

        self._arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming',
        )
        self._mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode',
        )
        self._distance_enable_client = self.create_client(
            SetBool,
            '/distance_control/enable',
        )
        self._local_takeoff_enable_client = self.create_client(
            SetBool,
            '/local_takeoff/enable',
        )

        self._state = MissionState.IDLE
        self._state_entered_s = time.monotonic()
        self._state_retry_count = 0
        self._pending_action: Optional[str] = None
        self._last_action_request_s: Dict[str, float] = {}
        self._action_attempts: Dict[str, int] = {}
        self._offboard_confirmed_once = False
        self._external_override_latched = False
        self._external_override_mode = ''
        self._offboard_entry_mode = ''
        self._abort_safe_complete_logged = False
        self._prestream_started_s: Optional[float] = None
        self._distance_control_started_s: Optional[float] = None
        self._target_hold_started_s: Optional[float] = None
        self._abort_reason = ''
        self._mission_started_s: Optional[float] = None
        self._last_log_sample_s = -math.inf
        self._log_file: Optional[IO[str]] = None
        self._log_writer = None
        self._log_path = ''

        self._mavros_received = False
        self._mavros_connected = False
        self._armed = False
        self._mode = ''
        self._distance_enabled = False
        self._target_reached = False
        self._local_takeoff_enabled = False
        self._local_takeoff_target_reached = False
        self._control_error_m: Optional[float] = None
        self._control_command_mps: Optional[float] = None
        self._distance_m: Optional[float] = None
        self._range_min_m = 0.0
        self._range_max_m = 0.0
        self._range_time_s: Optional[float] = None
        self._altitude_m: Optional[float] = None
        self._relative_altitude_m: Optional[float] = None
        self._pose_time_s: Optional[float] = None
        self._vertical_speed_mps: Optional[float] = None
        self._velocity_time_s: Optional[float] = None
        self._home_amsl_m: Optional[float] = None
        self._battery_remaining: Optional[float] = None
        self._battery_time_s: Optional[float] = None
        self._landed_state: Optional[int] = None
        self._extended_state_time_s: Optional[float] = None
        self._sensors_enabled: Optional[int] = None
        self._sensors_health: Optional[int] = None
        self._sys_status_time_s: Optional[float] = None
        self._autonomous_mission_state = 'IDLE'
        self._hover_window = StableWindow(self._hover_hold_s)

        self._timer = self.create_timer(
            1.0 / self._fsm_rate_hz,
            self._tick,
        )
        self.create_timer(0.5, self._publish_readiness)
        self._publish_state()
        self._publish_result('IDLE')
        self.get_logger().info(
            'Mission manager ready; call /mission/start to begin'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('validation_approved', False)
        self.declare_parameter('hover_hold_duration', 2.0)
        self.declare_parameter('prestream_duration', 2.0)
        self.declare_parameter('sensor_timeout', 0.3)
        self.declare_parameter('telemetry_timeout', 0.5)
        self.declare_parameter('status_timeout', 2.0)
        self.declare_parameter('minimum_battery_remaining', 0.30)
        self.declare_parameter('battery_id', 0)
        self.declare_parameter('require_on_ground_at_start', True)
        self.declare_parameter('require_enabled_sensors_healthy', True)
        self.declare_parameter('ignored_unhealthy_sensor_mask', 0)
        self.declare_parameter('distance_control_timeout', 20.0)
        self.declare_parameter('target_hold_confirm_duration', 0.2)
        self.declare_parameter('fsm_rate_hz', 20.0)
        self.declare_parameter('service_retry_interval', 1.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('loiter_mode', 'AUTO.LOITER')
        self.declare_parameter('land_mode', 'AUTO.LAND')
        self.declare_parameter('precheck_timeout', 5.0)
        self.declare_parameter('arming_timeout', 5.0)
        self.declare_parameter('takeoff_timeout', 5.0)
        self.declare_parameter('wait_hover_timeout', 15.0)
        self.declare_parameter('check_sensor_timeout', 3.0)
        self.declare_parameter('prestream_timeout', 6.0)
        self.declare_parameter('enable_timeout', 3.0)
        self.declare_parameter('offboard_timeout', 5.0)
        self.declare_parameter('target_hold_timeout', 3.0)
        self.declare_parameter('loiter_timeout', 5.0)
        self.declare_parameter('land_timeout', 5.0)
        self.declare_parameter('wait_disarm_timeout', 30.0)
        self.declare_parameter('abort_timeout', 30.0)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter(
            'log_directory',
            '~/da_daka_logs/distance_mission',
        )
        self.declare_parameter('telemetry_log_rate_hz', 10.0)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._validation_approved = bool(value('validation_approved'))
        self._hover_hold_s = float(value('hover_hold_duration'))
        self._prestream_s = float(value('prestream_duration'))
        self._sensor_timeout_s = float(value('sensor_timeout'))
        self._telemetry_timeout_s = float(value('telemetry_timeout'))
        self._status_timeout_s = float(value('status_timeout'))
        self._minimum_battery_remaining = float(
            value('minimum_battery_remaining')
        )
        self._battery_id = int(value('battery_id'))
        self._require_on_ground_at_start = bool(
            value('require_on_ground_at_start')
        )
        self._require_enabled_sensors_healthy = bool(
            value('require_enabled_sensors_healthy')
        )
        self._ignored_unhealthy_sensor_mask = int(
            value('ignored_unhealthy_sensor_mask')
        )
        self._control_timeout_s = float(
            value('distance_control_timeout')
        )
        self._target_hold_s = float(
            value('target_hold_confirm_duration')
        )
        self._fsm_rate_hz = float(value('fsm_rate_hz'))
        self._request_interval_s = float(value('service_retry_interval'))
        self._max_retries = int(value('max_retries'))
        self._loiter_mode = str(value('loiter_mode'))
        self._land_mode = str(value('land_mode'))
        self._enable_csv_logging = bool(value('enable_csv_logging'))
        self._log_directory = str(value('log_directory'))
        self._telemetry_log_rate_hz = float(
            value('telemetry_log_rate_hz')
        )
        self._timeouts = {
            MissionState.PRECHECK: float(value('precheck_timeout')),
            MissionState.ARMING: float(value('arming_timeout')),
            MissionState.TAKEOFF: float(value('takeoff_timeout')),
            MissionState.WAIT_HOVER: float(value('wait_hover_timeout')),
            MissionState.CHECK_SENSOR: float(
                value('check_sensor_timeout')
            ),
            MissionState.PRESTREAM_SETPOINT: float(
                value('prestream_timeout')
            ),
            MissionState.ENABLE_DISTANCE_CONTROL: float(
                value('enable_timeout')
            ),
            MissionState.ENTER_OFFBOARD: float(
                value('offboard_timeout')
            ),
            MissionState.DISTANCE_CONTROL: self._control_timeout_s,
            MissionState.TARGET_HOLD: float(
                value('target_hold_timeout')
            ),
            MissionState.LOITER_HANDOVER: float(
                value('loiter_timeout')
            ),
            MissionState.AUTO_LAND: float(value('land_timeout')),
            MissionState.WAIT_DISARM: float(
                value('wait_disarm_timeout')
            ),
            MissionState.ABORT: float(value('abort_timeout')),
        }
        positive_values = [
            self._hover_hold_s,
            self._prestream_s,
            self._sensor_timeout_s,
            self._telemetry_timeout_s,
            self._status_timeout_s,
            self._control_timeout_s,
            self._fsm_rate_hz,
            self._request_interval_s,
            self._telemetry_log_rate_hz,
        ]
        if any(item <= 0.0 for item in positive_values):
            raise ValueError('mission timing and distance values must be > 0')
        if not 0.0 <= self._minimum_battery_remaining <= 1.0:
            raise ValueError('minimum_battery_remaining must be within [0, 1]')
        if not 0 <= self._battery_id <= 9:
            raise ValueError('battery_id must be within [0, 9]')
        if self._max_retries < 0:
            raise ValueError('max_retries cannot be negative')
        if self._ignored_unhealthy_sensor_mask < 0:
            raise ValueError('ignored_unhealthy_sensor_mask cannot be negative')
        if not self._log_directory:
            raise ValueError('log_directory cannot be empty')

    def _start_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._state in self.ACTIVE_STATES or self._armed:
            response.success = False
            response.message = (
                f'mission cannot start from {self._state.name}; '
                f'armed={self._armed}'
            )
            return response
        if self._distance_enabled:
            response.success = False
            response.message = (
                'mission cannot start while distance control is enabled'
            )
            return response
        if self._local_takeoff_enabled:
            response.success = False
            response.message = (
                'mission cannot start while local takeoff control is enabled'
            )
            return response
        failures = self._preflight_failures(time.monotonic())
        if failures:
            response.success = False
            response.message = 'preflight failed: ' + '; '.join(failures)
            return response
        self._reset_mission()
        self._open_log()
        self._publish_result(f'RUNNING log={self._log_path}')
        self._transition(MissionState.PRECHECK)
        response.success = True
        response.message = 'mission start accepted'
        return response

    def _abort_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._abort('operator requested abort')
        response.success = True
        response.message = 'abort accepted; safe landing sequence active'
        return response

    def _range_callback(self, message: Range) -> None:
        if not math.isfinite(message.range):
            return
        self._distance_m = float(message.range)
        self._range_min_m = float(message.min_range)
        self._range_max_m = float(message.max_range)
        self._range_time_s = time.monotonic()

    def _enabled_callback(self, message: Bool) -> None:
        self._distance_enabled = bool(message.data)

    def _target_callback(self, message: Bool) -> None:
        self._target_reached = bool(message.data)

    def _local_takeoff_enabled_callback(self, message: Bool) -> None:
        self._local_takeoff_enabled = bool(message.data)

    def _local_takeoff_target_callback(self, message: Bool) -> None:
        self._local_takeoff_target_reached = bool(message.data)

    def _control_error_callback(self, message: Float32) -> None:
        value = float(message.data)
        if math.isfinite(value):
            self._control_error_m = value

    def _control_command_callback(self, message: Float32) -> None:
        value = float(message.data)
        if math.isfinite(value):
            self._control_command_mps = value

    def _mavros_state_callback(self, message: State) -> None:
        previous_mode = self._mode
        self._mavros_received = True
        self._mavros_connected = bool(message.connected)
        self._armed = bool(message.armed)
        self._mode = str(message.mode)
        if (
            self._is_external_land_override(
                self._state,
                previous_mode,
                self._mode,
                self._armed,
                self._land_mode,
            )
        ):
            self._external_override_latched = True
            self._external_override_mode = self._mode
            self._abort(
                f'external AUTO.LAND confirmed: '
                f'{previous_mode}->{self._mode}'
            )
            return
        if (
            self._is_external_mode_override(
                self._state,
                previous_mode,
                self._mode,
                self._offboard_confirmed_once,
            )
        ):
            self._external_override_latched = True
            self._external_override_mode = self._mode
            self._abort(
                f'external PX4 mode override confirmed: '
                f'{previous_mode}->{self._mode}'
            )
            return
        if (
            self._external_override_latched
            and self._state == MissionState.ABORT
            and self._mode != 'OFFBOARD'
        ):
            self._external_override_mode = self._mode

    def _pose_callback(self, message: PoseStamped) -> None:
        altitude = float(message.pose.position.z)
        if math.isfinite(altitude):
            self._altitude_m = altitude
            self._pose_time_s = time.monotonic()

    def _velocity_callback(self, message: TwistStamped) -> None:
        speed = float(message.twist.linear.z)
        if math.isfinite(speed):
            self._vertical_speed_mps = speed
            self._velocity_time_s = time.monotonic()

    def _altitude_callback(self, message: Altitude) -> None:
        amsl = float(message.amsl)
        relative = float(message.relative)
        if math.isfinite(amsl) and math.isfinite(relative):
            self._home_amsl_m = amsl - relative
            self._relative_altitude_m = relative

    def _battery_callback(self, message: BatteryState) -> None:
        if message.location != f'id{self._battery_id}':
            return
        remaining = float(message.percentage)
        if math.isfinite(remaining) and 0.0 <= remaining <= 1.0:
            self._battery_remaining = remaining
            self._battery_time_s = time.monotonic()

    def _extended_state_callback(self, message: ExtendedState) -> None:
        self._landed_state = int(message.landed_state)
        self._extended_state_time_s = time.monotonic()

    def _sys_status_callback(self, message: SysStatus) -> None:
        self._sensors_enabled = int(message.sensors_enabled)
        self._sensors_health = int(message.sensors_health)
        self._sys_status_time_s = time.monotonic()

    def _autonomous_state_callback(self, message: String) -> None:
        self._autonomous_mission_state = str(message.data)

    def _reset_mission(self) -> None:
        self._close_log()
        self._pending_action = None
        self._last_action_request_s.clear()
        self._action_attempts.clear()
        self._offboard_confirmed_once = False
        self._external_override_latched = False
        self._external_override_mode = ''
        self._offboard_entry_mode = ''
        self._abort_safe_complete_logged = False
        self._prestream_started_s = None
        self._distance_control_started_s = None
        self._target_hold_started_s = None
        self._abort_reason = ''
        self._hover_window.reset()
        self._mission_started_s = time.monotonic()
        self._last_log_sample_s = -math.inf

    @classmethod
    def _is_external_mode_override(
        cls,
        state: MissionState,
        previous_mode: str,
        current_mode: str,
        offboard_confirmed_once: bool,
    ) -> bool:
        """Return true when PX4 leaves OFFBOARD during active control."""
        return (
            offboard_confirmed_once
            and state in cls.OFFBOARD_CONTROL_STATES
            and previous_mode == 'OFFBOARD'
            and current_mode != 'OFFBOARD'
        )

    @classmethod
    def _is_external_land_override(
        cls,
        state: MissionState,
        previous_mode: str,
        current_mode: str,
        armed: bool,
        land_mode: str,
    ) -> bool:
        """Return true for an operator/PX4 LAND during an active mission."""
        return (
            armed
            and state in cls.ACTIVE_STATES
            and state not in {
                MissionState.AUTO_LAND,
                MissionState.WAIT_DISARM,
            }
            and previous_mode != land_mode
            and current_mode == land_mode
        )

    @staticmethod
    def _handover_mode_is_expected(
        state: MissionState,
        current_mode: str,
        loiter_mode: str,
        offboard_entry_mode: str,
    ) -> bool:
        """Accept only modes commanded by this FSM during handover states."""
        if state == MissionState.CHECK_SENSOR:
            return current_mode in {'OFFBOARD', loiter_mode}
        if state == MissionState.ENABLE_DISTANCE_CONTROL:
            return current_mode == loiter_mode
        if state == MissionState.ENTER_OFFBOARD:
            return current_mode in {'OFFBOARD', offboard_entry_mode}
        return True

    def _transition(self, next_state: MissionState) -> None:
        if next_state == self._state:
            return
        previous = self._state
        self._state = next_state
        self._state_entered_s = time.monotonic()
        self._state_retry_count = 0
        self._pending_action = None
        self._action_attempts.clear()
        if next_state in {
            MissionState.PRESTREAM_SETPOINT,
            MissionState.ENABLE_DISTANCE_CONTROL,
        }:
            self._prestream_started_s = None
        if next_state == MissionState.ENTER_OFFBOARD:
            self._offboard_entry_mode = self._mode
        if next_state == MissionState.DISTANCE_CONTROL:
            if self._distance_control_started_s is None:
                self._distance_control_started_s = time.monotonic()
        if next_state == MissionState.TARGET_HOLD:
            self._target_hold_started_s = None
        self._publish_state()
        self._write_log_row(f'STATE:{previous.name}->{next_state.name}')
        self.get_logger().info(
            f'FSM {previous.name} -> {next_state.name}'
        )

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self._state.name))

    def _publish_result(self, text: str) -> None:
        self._result_publisher.publish(String(data=text))
        self._write_log_row(f'RESULT:{text}')

    def _sensor_healthy(self, now_s: float) -> bool:
        if self._distance_m is None or self._range_time_s is None:
            return False
        if now_s - self._range_time_s > self._sensor_timeout_s:
            return False
        if not math.isfinite(self._distance_m) or self._distance_m <= 0.0:
            return False
        if self._range_min_m > 0.0 and self._distance_m < self._range_min_m:
            return False
        if self._range_max_m > 0.0 and self._distance_m > self._range_max_m:
            return False
        return True

    def _flight_data_healthy(self, now_s: float) -> bool:
        if not self._mavros_received or not self._mavros_connected:
            return False
        if self._pose_time_s is None or self._velocity_time_s is None:
            return False
        return (
            now_s - self._pose_time_s <= self._telemetry_timeout_s
            and now_s - self._velocity_time_s <= self._telemetry_timeout_s
        )

    def _precheck_ready(self, now_s: float) -> bool:
        return not self._preflight_failures(now_s)

    def _preflight_failures(self, now_s: float) -> list[str]:
        failures = []
        if not self._validation_approved:
            failures.append('flight validation approval is locked')
        if not self._flight_data_healthy(now_s):
            failures.append('MAVROS pose or velocity telemetry unavailable')
        if not self._sensor_healthy(now_s):
            failures.append('distance sensor unavailable or invalid')
        failures.extend(
            status_failures(
                now_s=now_s,
                timeout_s=self._status_timeout_s,
                battery_remaining=self._battery_remaining,
                battery_time_s=self._battery_time_s,
                minimum_battery_remaining=self._minimum_battery_remaining,
                landed_state=self._landed_state,
                extended_state_time_s=self._extended_state_time_s,
                require_on_ground=self._require_on_ground_at_start,
                sensors_enabled=self._sensors_enabled,
                sensors_health=self._sensors_health,
                sys_status_time_s=self._sys_status_time_s,
                require_enabled_sensors_healthy=(
                    self._require_enabled_sensors_healthy
                ),
                ignored_unhealthy_sensor_mask=(
                    self._ignored_unhealthy_sensor_mask
                ),
            )
        )
        inactive = {'', 'IDLE', 'COMPLETE', 'ABORT'}
        if self._autonomous_mission_state not in inactive:
            failures.append(
                'autonomous cleaning mission active '
                f'({self._autonomous_mission_state})'
            )
        velocity_publishers = self.count_publishers(
            '/mavros/setpoint_velocity/cmd_vel'
        )
        position_publishers = self.count_publishers(
            '/mavros/setpoint_position/local'
        )
        if velocity_publishers != 1:
            failures.append(
                'expected validation controller to be the only MAVROS '
                f'velocity publisher ({velocity_publishers} found)'
            )
        if position_publishers != 0:
            failures.append(
                'validation requires no MAVROS position publisher '
                f'({position_publishers} found)'
            )
        return failures

    def _publish_readiness(self) -> None:
        now_s = time.monotonic()
        failures = self._preflight_failures(now_s)
        inactive = self._state in {
            MissionState.IDLE,
            MissionState.COMPLETE,
            MissionState.ABORT,
        }
        if not inactive:
            failures.insert(0, f'validation active ({self._state.name})')
        payload = {
            'ready': inactive and not self._armed and not failures,
            'failures': failures,
            'validation_approved': self._validation_approved,
            'profile': 'TETHERED_1M_DISTANCE',
            'takeoff_distance_m': 1.1,
            'target_distance_m': 1.0,
            'minimum_battery_remaining': self._minimum_battery_remaining,
        }
        self._readiness_publisher.publish(
            String(data=json.dumps(payload, separators=(',', ':')))
        )

    def _inflight_status_failures(self, now_s: float) -> list[str]:
        return status_failures(
            now_s=now_s,
            timeout_s=self._status_timeout_s,
            battery_remaining=self._battery_remaining,
            battery_time_s=self._battery_time_s,
            minimum_battery_remaining=self._minimum_battery_remaining,
            landed_state=self._landed_state,
            extended_state_time_s=self._extended_state_time_s,
            require_on_ground=False,
            sensors_enabled=self._sensors_enabled,
            sensors_health=self._sensors_health,
            sys_status_time_s=self._sys_status_time_s,
            require_enabled_sensors_healthy=(
                self._require_enabled_sensors_healthy
            ),
            ignored_unhealthy_sensor_mask=(
                self._ignored_unhealthy_sensor_mask
            ),
        )

    def _tick(self) -> None:
        now_s = time.monotonic()
        self._write_periodic_log(now_s)
        if self._state in {MissionState.IDLE, MissionState.COMPLETE}:
            return
        if self._state == MissionState.ABORT:
            self._tick_abort(now_s)
            return

        if (
            self._state in self.ACTIVE_STATES
            and self._state != MissionState.PRECHECK
            and not self._mavros_connected
        ):
            self._abort('MAVROS disconnected')
            return
        if (
            self._state in self.ARMED_REQUIRED_STATES
            and (status_errors := self._inflight_status_failures(now_s))
        ):
            self._abort('PX4 status unhealthy: ' + '; '.join(status_errors))
            return
        if (
            self._state in self.SENSOR_CRITICAL_STATES
            and not self._sensor_healthy(now_s)
        ):
            self._abort('distance sensor timeout or invalid range')
            return
        if (
            self._state in self.ARMED_REQUIRED_STATES
            and not self._armed
        ):
            self._abort(f'vehicle disarmed unexpectedly in {self._state.name}')
            return

        handlers = {
            MissionState.PRECHECK: self._tick_precheck,
            MissionState.ARMING: self._tick_arming,
            MissionState.TAKEOFF: self._tick_takeoff,
            MissionState.WAIT_HOVER: self._tick_wait_hover,
            MissionState.CHECK_SENSOR: self._tick_check_sensor,
            MissionState.PRESTREAM_SETPOINT: self._tick_prestream,
            MissionState.ENABLE_DISTANCE_CONTROL: self._tick_enable,
            MissionState.ENTER_OFFBOARD: self._tick_offboard,
            MissionState.DISTANCE_CONTROL: self._tick_distance_control,
            MissionState.TARGET_HOLD: self._tick_target_hold,
            MissionState.LOITER_HANDOVER: self._tick_loiter,
            MissionState.AUTO_LAND: self._tick_auto_land,
            MissionState.WAIT_DISARM: self._tick_wait_disarm,
        }
        handler = handlers.get(self._state)
        if handler is not None:
            handler(now_s)
        self._check_state_timeout(now_s)

    def _tick_precheck(self, now_s: float) -> None:
        if self._precheck_ready(now_s):
            self._transition(MissionState.ARMING)

    def _tick_arming(self, _now_s: float) -> None:
        if self._armed:
            self._transition(MissionState.TAKEOFF)
            return
        request = CommandBool.Request()
        request.value = True
        self._request_action(
            'arm',
            self._arming_client,
            request,
            lambda response: bool(response.success),
        )

    def _tick_takeoff(self, _now_s: float) -> None:
        """Enable the configured takeoff reference and zero prestream."""
        if self._local_takeoff_enabled:
            self._transition(MissionState.PRESTREAM_SETPOINT)
            return
        self._request_local_takeoff_enable(True, 'local_takeoff_enable')

    def _tick_wait_hover(self, now_s: float) -> None:
        if not self._local_takeoff_enabled:
            self._abort('local takeoff controller unexpectedly disabled')
            return
        if self._mode != 'OFFBOARD':
            self._abort(f'OFFBOARD lost during takeoff: {self._mode}')
            return
        if self._hover_window.update(
            self._local_takeoff_target_reached,
            now_s,
        ):
            self._transition(MissionState.CHECK_SENSOR)

    def _tick_check_sensor(self, now_s: float) -> None:
        if not self._sensor_healthy(now_s):
            return
        # Leave OFFBOARD before changing the sole setpoint publisher's mode.
        # This avoids an OFFBOARD setpoint gap between two service calls.
        if self._mode == 'OFFBOARD':
            self._request_mode(self._loiter_mode)
            return
        if not self._handover_mode_is_expected(
            self._state,
            self._mode,
            self._loiter_mode,
            self._offboard_entry_mode,
        ):
            self._abort_for_external_mode(
                f'QGC/PX4 mode override during LOITER handover: {self._mode}'
            )
            return
        self._transition(MissionState.ENABLE_DISTANCE_CONTROL)

    def _tick_prestream(self, now_s: float) -> None:
        if not self._local_takeoff_enabled:
            self._prestream_started_s = None
            self._request_local_takeoff_enable(
                True,
                'local_takeoff_enable',
            )
            return
        if self._prestream_started_s is None:
            self._prestream_started_s = now_s
            self.get_logger().info(
                'Takeoff controller enabled; zero prestream started'
            )
        if now_s - self._prestream_started_s >= self._prestream_s:
            self._transition(MissionState.ENTER_OFFBOARD)

    def _tick_enable(self, now_s: float) -> None:
        # The vertical controller rejects simultaneous modes. Disable takeoff
        # control first, then enable hold control while PX4 is in AUTO.LOITER.
        if not self._handover_mode_is_expected(
            self._state,
            self._mode,
            self._loiter_mode,
            self._offboard_entry_mode,
        ):
            self._abort_for_external_mode(
                f'QGC/PX4 mode override during controller handover: {self._mode}'
            )
            return
        if self._local_takeoff_enabled:
            self._request_local_takeoff_enable(
                False,
                'local_takeoff_disable',
            )
            return
        if self._distance_enabled:
            if self._prestream_started_s is None:
                self._prestream_started_s = now_s
                self.get_logger().info(
                    'LiDAR distance setpoint prestream started'
                )
            if now_s - self._prestream_started_s >= self._prestream_s:
                self._transition(MissionState.ENTER_OFFBOARD)
            return
        request = SetBool.Request()
        request.data = True
        self._request_action(
            'distance_enable',
            self._distance_enable_client,
            request,
            lambda response: bool(response.success),
        )

    def _tick_offboard(self, _now_s: float) -> None:
        if self._mode == 'OFFBOARD':
            self._offboard_confirmed_once = True
            if self._distance_enabled:
                self._distance_control_started_s = time.monotonic()
                self._transition(MissionState.DISTANCE_CONTROL)
            else:
                self._transition(MissionState.WAIT_HOVER)
            return
        if (
            self._mode == self._land_mode
        ):
            self._external_override_latched = True
            self._external_override_mode = self._mode
            self._abort(
                'external AUTO.LAND confirmed before OFFBOARD entry'
            )
            return
        if not self._handover_mode_is_expected(
            self._state,
            self._mode,
            self._loiter_mode,
            self._offboard_entry_mode,
        ):
            self._abort_for_external_mode(
                f'QGC/PX4 mode override before OFFBOARD entry: {self._mode}'
            )
            return
        self._request_mode('OFFBOARD')

    def _tick_distance_control(self, now_s: float) -> None:
        if not self._distance_enabled:
            self._abort('distance controller unexpectedly disabled')
            return
        if self._mode != 'OFFBOARD':
            self._external_override_latched = True
            self._external_override_mode = self._mode
            self._abort(
                f'OFFBOARD lost; respecting actual mode={self._mode}'
            )
            return
        if (
            self._distance_control_started_s is not None
            and now_s - self._distance_control_started_s
            > self._control_timeout_s
        ):
            self._abort('distance control exceeded 20 second timeout')
            return
        if self._target_reached:
            self._transition(MissionState.TARGET_HOLD)

    def _tick_target_hold(self, now_s: float) -> None:
        if self._mode != 'OFFBOARD':
            self._external_override_latched = True
            self._external_override_mode = self._mode
            self._abort(
                f'OFFBOARD lost during target hold; '
                f'respecting actual mode={self._mode}'
            )
            return
        if not self._target_reached:
            self._target_hold_started_s = None
            self._transition(MissionState.DISTANCE_CONTROL)
            return
        if self._target_hold_started_s is None:
            self._target_hold_started_s = now_s
        if now_s - self._target_hold_started_s >= self._target_hold_s:
            self._transition(MissionState.LOITER_HANDOVER)

    def _tick_loiter(self, _now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        if self._distance_enabled:
            self._request_distance_enable(False, 'distance_disable')
            return
        self._transition(MissionState.AUTO_LAND)

    def _tick_auto_land(self, _now_s: float) -> None:
        if self._mode == self._land_mode:
            self._transition(MissionState.WAIT_DISARM)
            return
        self._request_mode(self._land_mode)

    def _tick_wait_disarm(self, _now_s: float) -> None:
        if self._armed:
            return
        self._transition(MissionState.COMPLETE)
        self._publish_result(f'SUCCESS log={self._log_path}')
        self._close_log()
        self.get_logger().info('Mission complete; vehicle disarmed')

    def _request_mode(self, mode: str) -> None:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self._request_action(
            f'mode:{mode}',
            self._mode_client,
            request,
            lambda response: bool(response.mode_sent),
        )

    def _request_distance_enable(self, enabled: bool, action: str) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._request_action(
            action,
            self._distance_enable_client,
            request,
            lambda response: bool(response.success),
        )

    def _request_local_takeoff_enable(
        self,
        enabled: bool,
        action: str,
    ) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._request_action(
            action,
            self._local_takeoff_enable_client,
            request,
            lambda response: bool(response.success),
        )

    def _request_action(
        self,
        action: str,
        client,
        request,
        accepted: Callable[[object], bool],
    ) -> None:
        now_s = time.monotonic()
        if self._pending_action is not None:
            return
        last_request = self._last_action_request_s.get(action, -math.inf)
        if now_s - last_request < self._request_interval_s:
            return
        attempts = self._action_attempts.get(action, 0)
        if attempts >= self._max_retries + 1:
            self._abort(f'{action} exceeded retry limit')
            return
        if not client.service_is_ready():
            self._last_action_request_s[action] = now_s
            self._action_attempts[action] = attempts + 1
            self.get_logger().warning(f'{action} service unavailable')
            return

        self._pending_action = action
        self._last_action_request_s[action] = now_s
        self._action_attempts[action] = attempts + 1
        future = client.call_async(request)
        future.add_done_callback(
            lambda completed: self._action_done(
                action,
                completed,
                accepted,
            )
        )
        self.get_logger().info(
            f'Requested {action} '
            f'(attempt {self._action_attempts[action]})'
        )

    def _action_done(
        self,
        action: str,
        future,
        accepted: Callable[[object], bool],
    ) -> None:
        if self._pending_action == action:
            self._pending_action = None
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - transport failure
            self.get_logger().error(f'{action} service error: {error}')
            return
        if not accepted(response):
            self.get_logger().warning(
                f'{action} request rejected; waiting for state/retry'
            )

    def _check_state_timeout(self, now_s: float) -> None:
        timeout_s = self._timeouts.get(self._state)
        if timeout_s is None:
            return
        if now_s - self._state_entered_s <= timeout_s:
            return
        if self._state == MissionState.DISTANCE_CONTROL:
            self._abort('distance control exceeded 20 second timeout')
            return
        if self._state_retry_count < self._max_retries:
            self._state_retry_count += 1
            self._state_entered_s = now_s
            self._pending_action = None
            self._action_attempts.clear()
            self._hover_window.reset()
            self.get_logger().warning(
                f'{self._state.name} timeout; '
                f'retry {self._state_retry_count}/{self._max_retries}'
            )
            return
        self._abort(
            f'{self._state.name} timed out after '
            f'{self._max_retries} retries'
        )

    def _abort(self, reason: str) -> None:
        if self._state == MissionState.ABORT:
            return
        self._abort_reason = reason
        self._transition(MissionState.ABORT)
        self._publish_result(
            f'ABORTED: {reason}; log={self._log_path}'
        )
        self.get_logger().error(f'Mission abort: {reason}')

    def _abort_for_external_mode(self, reason: str) -> None:
        """Latch a confirmed QGC/PX4 mode and never request OFFBOARD again."""
        self._external_override_latched = True
        self._external_override_mode = self._mode
        self._abort(reason)

    def _tick_abort(self, now_s: float) -> None:
        if not self._armed:
            if self._distance_enabled:
                self._request_distance_enable(
                    False,
                    'abort_distance_disable',
                )
                return
            if self._local_takeoff_enabled:
                self._request_local_takeoff_enable(
                    False,
                    'abort_local_takeoff_disable',
                )
                return
            if not self._abort_safe_complete_logged:
                self._write_log_row('ABORT_SAFE_COMPLETE')
                self._abort_safe_complete_logged = True
            self._close_log()
            return

        if (
            self._external_override_latched
            and self._mode != 'OFFBOARD'
        ):
            # PX4 has confirmed the QGC/failsafe-selected mode. Never fight it.
            # Never fight that mode or re-enter OFFBOARD. Setpoints may stop
            # only after the non-OFFBOARD state has been observed here.
            if self._distance_enabled:
                self._request_distance_enable(
                    False,
                    'override_distance_disable',
                )
                return
            if self._local_takeoff_enabled:
                self._request_local_takeoff_enable(
                    False,
                    'override_local_takeoff_disable',
                )
                return
            if not self._abort_safe_complete_logged:
                self._write_log_row(
                    f'EXTERNAL_MODE_OVERRIDE:{self._external_override_mode}'
                )
                self._publish_result(
                    'ABORTED: external mode override retained; '
                    f'mode={self._external_override_mode}; '
                    'QGC/PX4 owns recovery'
                )
                self._abort_safe_complete_logged = True
            return

        if self._mode != self._land_mode:
            self._request_mode(self._land_mode)
            return

        # Keep OFFBOARD setpoints alive until PX4 confirms AUTO.LAND.
        if self._distance_enabled:
            self._request_distance_enable(
                False,
                'abort_distance_disable',
            )
            return
        if self._local_takeoff_enabled:
            self._request_local_takeoff_enable(
                False,
                'abort_local_takeoff_disable',
            )

        timeout_s = self._timeouts[MissionState.ABORT]
        if now_s - self._state_entered_s > timeout_s:
            self.get_logger().error(
                'ABORT timeout: AUTO.LAND remains active; '
                'use QGC/PX4 emergency procedure'
            )
            self._state_entered_s = now_s

    def _open_log(self) -> None:
        if not self._enable_csv_logging:
            self._log_path = 'disabled'
            return
        directory = Path(self._log_directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path = directory / f'distance_mission_{timestamp}.csv'
        self._log_file = path.open('x', newline='', encoding='utf-8')
        self._log_writer = csv.writer(self._log_file)
        self._log_path = str(path)
        self._log_writer.writerow(
            [
                'wall_time',
                'elapsed_s',
                'event',
                'mission_state',
                'mavros_connected',
                'armed',
                'px4_mode',
                'distance_m',
                'control_error_m',
                'vertical_speed_command_mps',
                'altitude_m',
                'relative_altitude_m',
                'vertical_speed_mps',
                'distance_enabled',
                'target_reached',
                'external_override_latched',
                'external_override_mode',
            ]
        )
        self._log_file.flush()
        self.get_logger().info(f'Mission CSV log: {self._log_path}')

    def _write_periodic_log(self, now_s: float) -> None:
        period_s = 1.0 / self._telemetry_log_rate_hz
        if now_s - self._last_log_sample_s < period_s:
            return
        self._last_log_sample_s = now_s
        self._write_log_row('')

    @staticmethod
    def _optional_number(value: Optional[float]):
        if value is None:
            return ''
        return f'{value:.6f}'

    def _write_log_row(self, event: str) -> None:
        if self._log_writer is None or self._log_file is None:
            return
        now_s = time.monotonic()
        elapsed_s = (
            now_s - self._mission_started_s
            if self._mission_started_s is not None
            else 0.0
        )
        self._log_writer.writerow(
            [
                datetime.now().astimezone().isoformat(timespec='milliseconds'),
                f'{elapsed_s:.3f}',
                event,
                self._state.name,
                self._mavros_connected,
                self._armed,
                self._mode,
                self._optional_number(self._distance_m),
                self._optional_number(self._control_error_m),
                self._optional_number(self._control_command_mps),
                self._optional_number(self._altitude_m),
                self._optional_number(self._relative_altitude_m),
                self._optional_number(self._vertical_speed_mps),
                self._distance_enabled,
                self._target_reached,
                self._external_override_latched,
                self._external_override_mode,
            ]
        )
        self._log_file.flush()

    def _close_log(self) -> None:
        if self._log_file is None:
            return
        self._log_file.flush()
        self._log_file.close()
        self._log_file = None
        self._log_writer = None


def main(args=None) -> None:
    """Run the mission manager node."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
