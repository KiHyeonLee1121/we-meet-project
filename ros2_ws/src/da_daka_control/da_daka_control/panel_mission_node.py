"""Run an independent Local-Z panel-movement flight through MAVROS."""

import math
import time
from typing import Callable, Optional

from da_daka_control.mission_manager_node import status_failures
from da_daka_control.panel_mission_fsm import (
    advance_position_setpoint,
    control_ownership_failures,
    PanelMissionFsm,
    PanelMissionState,
    PanelRoute,
    RelativeWaypoint,
    StableArrival,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class PanelMissionNode(Node):
    """Coordinate a complete arm, Local-Z takeoff, route, and landing."""

    OFFBOARD_STATES = {
        PanelMissionState.TAKEOFF_HOLD,
        PanelMissionState.MOVE_TO_PANEL,
        PanelMissionState.HOLD_PANEL,
    }
    ACTIVE_STATES = PanelMissionFsm.ACTIVE_STATES
    ON_GROUND = 1

    def __init__(self) -> None:
        super().__init__('panel_mission')
        self._declare_parameters()
        self._load_parameters()
        self._fsm = PanelMissionFsm(self._make_route())
        self._arrival = StableArrival(
            self._arrival_tolerance_m,
            self._arrival_max_speed_mps,
            self._arrival_stable_s,
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, '/panel_mission/state', latched_qos
        )
        self._result_publisher = self.create_publisher(
            String, '/panel_mission/result', latched_qos
        )
        self._setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, '/panel_mission/start', self._start)
        self.create_service(Trigger, '/panel_mission/abort', self._abort)

        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self._extended_state,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SysStatus, '/mavros/sys_status', self._sys_status, 10
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/enabled',
            self._takeoff_enabled_state,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/target_reached',
            self._takeoff_target_state,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/distance_control/enabled',
            self._distance_control_state,
            latched_qos,
        )
        self.create_subscription(
            String,
            '/vertical_control/mode',
            self._vertical_control_state,
            latched_qos,
        )
        self.create_subscription(
            String,
            '/mission/state',
            self._distance_mission_state_callback,
            latched_qos,
        )

        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._takeoff_client = self.create_client(
            SetBool, '/local_takeoff/enable'
        )

        self._connected = False
        self._armed = False
        self._mode = ''
        self._landed_state: Optional[int] = None
        self._extended_state_time_s: Optional[float] = None
        self._pose_xyz: Optional[tuple[float, float, float]] = None
        self._orientation = None
        self._velocity_xyz: Optional[tuple[float, float, float]] = None
        self._pose_time_s: Optional[float] = None
        self._velocity_time_s: Optional[float] = None
        self._battery_remaining: Optional[float] = None
        self._battery_time_s: Optional[float] = None
        self._sensors_enabled: Optional[int] = None
        self._sensors_health: Optional[int] = None
        self._sys_status_time_s: Optional[float] = None
        self._ground_xyz: Optional[tuple[float, float, float]] = None
        self._launch_xyz: Optional[tuple[float, float, float]] = None
        self._local_takeoff_enabled = False
        self._local_takeoff_reached = False
        self._distance_control_enabled = False
        self._vertical_control_mode = ''
        self._distance_mission_state = ''
        self._state_started_s = time.monotonic()
        self._prestream_started_s: Optional[float] = None
        self._hold_started_s: Optional[float] = None
        self._command_xyz: Optional[tuple[float, float, float]] = None
        self._last_setpoint_s = time.monotonic()
        self._pending_action: Optional[str] = None
        self._last_action_s: dict[str, float] = {}
        self._abort_land_requested = False
        self._publish_state()
        self._publish_result('IDLE')
        self.create_timer(1.0 / self._tick_rate_hz, self._tick)
        self.get_logger().info(
            'Panel mission ready in IDLE; explicit start service required; '
            f'configuration_approved={self._configuration_approved}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('configuration_approved', False)
        self.declare_parameter('waypoint_x_m', [0.0])
        self.declare_parameter('waypoint_y_m', [0.0])
        self.declare_parameter('takeoff_height_m', 1.1)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('arrival_max_speed_mps', 0.10)
        self.declare_parameter('arrival_stable_s', 2.0)
        self.declare_parameter('maximum_horizontal_setpoint_speed_mps', 0.30)
        self.declare_parameter('maximum_vertical_setpoint_speed_mps', 0.20)
        self.declare_parameter('panel_hold_s', 3.0)
        self.declare_parameter('prestream_s', 2.0)
        self.declare_parameter('telemetry_timeout_s', 0.5)
        self.declare_parameter('status_timeout_s', 3.0)
        self.declare_parameter('minimum_battery_remaining', 0.15)
        self.declare_parameter('battery_id', 0)
        self.declare_parameter('require_enabled_sensors_healthy', True)
        self.declare_parameter('ignored_unhealthy_sensor_mask', 0)
        self.declare_parameter('loiter_mode', 'AUTO.LOITER')
        self.declare_parameter('land_mode', 'AUTO.LAND')
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('action_retry_s', 1.0)
        self.declare_parameter('state_timeout_s', 30.0)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._configuration_approved = bool(value('configuration_approved'))
        self._waypoint_x = [float(item) for item in value('waypoint_x_m')]
        self._waypoint_y = [float(item) for item in value('waypoint_y_m')]
        self._takeoff_height_m = float(value('takeoff_height_m'))
        self._arrival_tolerance_m = float(value('arrival_tolerance_m'))
        self._arrival_max_speed_mps = float(value('arrival_max_speed_mps'))
        self._arrival_stable_s = float(value('arrival_stable_s'))
        self._maximum_horizontal_setpoint_speed_mps = float(
            value('maximum_horizontal_setpoint_speed_mps')
        )
        self._maximum_vertical_setpoint_speed_mps = float(
            value('maximum_vertical_setpoint_speed_mps')
        )
        self._panel_hold_s = float(value('panel_hold_s'))
        self._prestream_s = float(value('prestream_s'))
        self._telemetry_timeout_s = float(value('telemetry_timeout_s'))
        self._status_timeout_s = float(value('status_timeout_s'))
        self._minimum_battery = float(value('minimum_battery_remaining'))
        self._battery_id = int(value('battery_id'))
        self._require_health = bool(value('require_enabled_sensors_healthy'))
        self._ignored_health = int(value('ignored_unhealthy_sensor_mask'))
        self._loiter_mode = str(value('loiter_mode'))
        self._land_mode = str(value('land_mode'))
        self._tick_rate_hz = float(value('tick_rate_hz'))
        self._action_retry_s = float(value('action_retry_s'))
        self._state_timeout_s = float(value('state_timeout_s'))
        if len(self._waypoint_x) != len(self._waypoint_y):
            raise ValueError('waypoint_x_m and waypoint_y_m lengths differ')
        positive = (
            self._takeoff_height_m,
            self._arrival_tolerance_m,
            self._arrival_stable_s,
            self._maximum_horizontal_setpoint_speed_mps,
            self._maximum_vertical_setpoint_speed_mps,
            self._panel_hold_s,
            self._prestream_s,
            self._telemetry_timeout_s,
            self._status_timeout_s,
            self._tick_rate_hz,
            self._action_retry_s,
            self._state_timeout_s,
        )
        if any(item <= 0.0 for item in positive):
            raise ValueError('panel mission timing and distance values must be positive')
        if not 0.0 <= self._minimum_battery <= 1.0:
            raise ValueError('minimum_battery_remaining must be in [0, 1]')
        if not 0 <= self._battery_id <= 9:
            raise ValueError('battery_id must be within [0, 9]')
        if self._ignored_health < 0:
            raise ValueError('ignored_unhealthy_sensor_mask cannot be negative')

    def _make_route(self) -> PanelRoute:
        return PanelRoute(
            [
                RelativeWaypoint(x_m, y_m)
                for x_m, y_m in zip(self._waypoint_x, self._waypoint_y)
            ]
        )

    def _start(self, _request, response):
        if self._fsm.active:
            response.success = False
            response.message = f'already active: {self._fsm.state.name}'
            return response
        if not self._configuration_approved:
            response.success = False
            response.message = 'panel coordinates are not approved'
            return response
        self._fsm.start()
        self._state_started_s = time.monotonic()
        self._launch_xyz = None
        self._arrival.reset()
        self._abort_land_requested = False
        self._command_xyz = None
        self._publish_state()
        self._publish_result('RUNNING')
        response.success = True
        response.message = 'panel mission accepted'
        return response

    def _abort(self, _request, response):
        if not self._fsm.active:
            response.success = True
            response.message = 'mission is not active'
            return response
        self._fail('operator abort', request_land=True)
        response.success = True
        response.message = 'abort accepted'
        return response

    def _state(self, message: State) -> None:
        self._connected = bool(message.connected)
        self._armed = bool(message.armed)
        self._mode = str(message.mode)

    def _extended_state(self, message: ExtendedState) -> None:
        self._landed_state = int(message.landed_state)
        self._extended_state_time_s = time.monotonic()
        self._capture_ground()

    def _pose(self, message: PoseStamped) -> None:
        xyz = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        if all(math.isfinite(item) for item in xyz):
            self._pose_xyz = xyz
            self._orientation = message.pose.orientation
            self._pose_time_s = time.monotonic()
            self._capture_ground()

    def _velocity(self, message: TwistStamped) -> None:
        xyz = (
            float(message.twist.linear.x),
            float(message.twist.linear.y),
            float(message.twist.linear.z),
        )
        if all(math.isfinite(item) for item in xyz):
            self._velocity_xyz = xyz
            self._velocity_time_s = time.monotonic()

    def _battery(self, message: BatteryState) -> None:
        if message.location != f'id{self._battery_id}':
            return
        remaining = float(message.percentage)
        self._battery_remaining = remaining if math.isfinite(remaining) else None
        self._battery_time_s = time.monotonic()

    def _sys_status(self, message: SysStatus) -> None:
        self._sensors_enabled = int(message.sensors_enabled)
        self._sensors_health = int(message.sensors_health)
        self._sys_status_time_s = time.monotonic()

    def _takeoff_enabled_state(self, message: Bool) -> None:
        self._local_takeoff_enabled = bool(message.data)

    def _takeoff_target_state(self, message: Bool) -> None:
        self._local_takeoff_reached = bool(message.data)

    def _distance_control_state(self, message: Bool) -> None:
        self._distance_control_enabled = bool(message.data)

    def _vertical_control_state(self, message: String) -> None:
        self._vertical_control_mode = str(message.data)

    def _distance_mission_state_callback(self, message: String) -> None:
        self._distance_mission_state = str(message.data)

    def _capture_ground(self) -> None:
        if not self._armed and self._landed_state == self.ON_GROUND:
            if self._pose_xyz is not None:
                self._ground_xyz = self._pose_xyz

    def _telemetry_fresh(self, now_s: float) -> bool:
        return (
            self._pose_time_s is not None
            and self._velocity_time_s is not None
            and now_s - self._pose_time_s <= self._telemetry_timeout_s
            and now_s - self._velocity_time_s <= self._telemetry_timeout_s
        )

    def _precheck_failures(self, now_s: float) -> list[str]:
        failures = status_failures(
            now_s=now_s,
            timeout_s=self._status_timeout_s,
            battery_remaining=self._battery_remaining,
            battery_time_s=self._battery_time_s,
            minimum_battery_remaining=self._minimum_battery,
            landed_state=self._landed_state,
            extended_state_time_s=self._extended_state_time_s,
            require_on_ground=True,
            sensors_enabled=self._sensors_enabled,
            sensors_health=self._sensors_health,
            sys_status_time_s=self._sys_status_time_s,
            require_enabled_sensors_healthy=self._require_health,
            ignored_unhealthy_sensor_mask=self._ignored_health,
        )
        if not self._connected:
            failures.append('MAVROS disconnected')
        if self._armed:
            failures.append('vehicle already armed')
        if not self._telemetry_fresh(now_s):
            failures.append('local pose or velocity telemetry stale')
        if self._ground_xyz is None:
            failures.append('launch Local XYZ reference unavailable')
        if self._mode == self._land_mode:
            failures.append('PX4 is already in AUTO.LAND')
        failures.extend(
            control_ownership_failures(
                distance_control_enabled=self._distance_control_enabled,
                vertical_control_mode=self._vertical_control_mode,
                distance_mission_publishers=self.count_publishers(
                    '/mission/state'
                ),
                distance_mission_state=self._distance_mission_state,
                velocity_setpoint_publishers=self.count_publishers(
                    '/mavros/setpoint_velocity/cmd_vel'
                ),
                position_setpoint_publishers=self.count_publishers(
                    '/mavros/setpoint_position/local'
                ),
            )
        )
        return failures

    def _transition(self, state: PanelMissionState, reason: str = '') -> None:
        previous = self._fsm.state
        self._fsm.transition(state, reason)
        self._state_started_s = time.monotonic()
        self._prestream_started_s = None
        self.get_logger().info(f'{previous.name} -> {state.name}: {self._fsm.reason}')
        self._publish_state()

    def _tick(self) -> None:
        now_s = time.monotonic()
        state = self._fsm.state
        if state in {PanelMissionState.IDLE, PanelMissionState.COMPLETE}:
            return
        if state == PanelMissionState.ABORT:
            self._tick_abort()
            return
        if (
            self._mode == self._land_mode
            and state not in {
                PanelMissionState.AUTO_LAND,
                PanelMissionState.WAIT_DISARM,
            }
        ):
            self._fail(
                'external AUTO.LAND confirmed; respecting PX4/QGC',
                request_land=False,
            )
            return
        if self._distance_control_enabled:
            self._fail(
                'LiDAR distance control enabled during panel mission',
                request_land=True,
            )
            return
        if self.count_publishers('/mavros/setpoint_velocity/cmd_vel') != 1:
            self._fail('vertical setpoint publisher ownership changed', True)
            return
        if self.count_publishers('/mavros/setpoint_position/local') != 1:
            self._fail('position setpoint publisher ownership changed', True)
            return
        if (
            state not in {
                PanelMissionState.PRECHECK,
                PanelMissionState.ARMING,
            }
            and not self._telemetry_fresh(now_s)
        ):
            self._fail('local pose or velocity telemetry timeout', True)
            return
        if now_s - self._state_started_s > self._state_timeout_s:
            self._fail(f'{state.name} timeout', request_land=True)
            return
        if state in self.OFFBOARD_STATES and self._mode != 'OFFBOARD':
            self._fail(
                f'QGC/PX4 mode override: {self._mode}',
                request_land=False,
            )
            return
        handlers = {
            PanelMissionState.PRECHECK: self._tick_precheck,
            PanelMissionState.ARMING: self._tick_arming,
            PanelMissionState.TAKEOFF: self._tick_takeoff,
            PanelMissionState.TAKEOFF_PRESTREAM: self._tick_takeoff_prestream,
            PanelMissionState.TAKEOFF_OFFBOARD: self._tick_takeoff_offboard,
            PanelMissionState.TAKEOFF_HOLD: self._tick_takeoff_hold,
            PanelMissionState.LOITER_HANDOVER: self._tick_loiter_handover,
            PanelMissionState.MOVE_PRESTREAM: self._tick_move_prestream,
            PanelMissionState.MOVE_OFFBOARD: self._tick_move_offboard,
            PanelMissionState.MOVE_TO_PANEL: self._tick_move,
            PanelMissionState.HOLD_PANEL: self._tick_hold,
            PanelMissionState.FINAL_LOITER: self._tick_final_loiter,
            PanelMissionState.AUTO_LAND: self._tick_land,
            PanelMissionState.WAIT_DISARM: self._tick_wait_disarm,
        }
        handler = handlers.get(state)
        if handler is not None:
            handler(now_s)

    def _tick_precheck(self, now_s: float) -> None:
        failures = self._precheck_failures(now_s)
        if failures:
            self._fail('preflight failed: ' + '; '.join(failures), False)
            return
        self._launch_xyz = self._ground_xyz
        self._transition(PanelMissionState.ARMING)

    def _tick_arming(self, _now_s: float) -> None:
        if self._armed:
            self._transition(PanelMissionState.TAKEOFF)
            return
        request = CommandBool.Request()
        request.value = True
        self._request('arm', self._arm_client, request, lambda r: r.success)

    def _tick_takeoff(self, _now_s: float) -> None:
        if self._local_takeoff_enabled:
            self._transition(PanelMissionState.TAKEOFF_PRESTREAM)
            return
        self._request_takeoff(True, 'takeoff_enable')

    def _tick_takeoff_prestream(self, now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before takeoff: {self._mode}',
                request_land=False,
            )
            return
        if not self._local_takeoff_enabled:
            self._fail('local takeoff controller disabled', True)
            return
        if self._prestream_started_s is None:
            self._prestream_started_s = now_s
        if now_s - self._prestream_started_s >= self._prestream_s:
            self._transition(PanelMissionState.TAKEOFF_OFFBOARD)

    def _tick_takeoff_offboard(self, _now_s: float) -> None:
        if self._mode == 'OFFBOARD':
            self._transition(PanelMissionState.TAKEOFF_HOLD)
            return
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before OFFBOARD: {self._mode}',
                request_land=False,
            )
            return
        self._request_mode('OFFBOARD')

    def _tick_takeoff_hold(self, _now_s: float) -> None:
        if self._local_takeoff_reached:
            self._transition(PanelMissionState.LOITER_HANDOVER)

    def _tick_loiter_handover(self, _now_s: float) -> None:
        if self._mode == self._land_mode:
            self._fail(
                'external AUTO.LAND during controller handover',
                request_land=False,
            )
            return
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        if self._local_takeoff_enabled:
            self._request_takeoff(False, 'takeoff_disable')
            return
        if self._pose_xyz is None:
            self._fail('local position unavailable at movement handover', True)
            return
        self._command_xyz = self._pose_xyz
        self._last_setpoint_s = time.monotonic()
        self._transition(PanelMissionState.MOVE_PRESTREAM)

    def _tick_move_prestream(self, now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before movement: {self._mode}',
                request_land=False,
            )
            return
        self._publish_command_setpoint()
        if self._prestream_started_s is None:
            self._prestream_started_s = now_s
        if now_s - self._prestream_started_s >= self._prestream_s:
            self._transition(PanelMissionState.MOVE_OFFBOARD)

    def _tick_move_offboard(self, _now_s: float) -> None:
        self._publish_command_setpoint()
        if self._mode == 'OFFBOARD':
            self._arrival.reset()
            self._transition(PanelMissionState.MOVE_TO_PANEL)
            return
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before movement OFFBOARD: {self._mode}',
                request_land=False,
            )
            return
        self._request_mode('OFFBOARD')

    def _tick_move(self, now_s: float) -> None:
        target = self._fsm.route.current
        self._advance_command_setpoint(target, now_s)
        self._publish_command_setpoint()
        error = self._position_error(target)
        speed = self._speed()
        if self._arrival.update(
            position_error_m=error,
            speed_mps=speed,
            now_s=now_s,
            telemetry_valid=self._telemetry_fresh(now_s),
        ):
            self._fsm.panel_reached()
            self._state_started_s = now_s
            self._hold_started_s = now_s
            self._publish_state()

    def _tick_hold(self, now_s: float) -> None:
        self._command_xyz = self._target_xyz(self._fsm.route.current)
        self._publish_command_setpoint()
        if self._hold_started_s is None:
            self._hold_started_s = now_s
        if now_s - self._hold_started_s >= self._panel_hold_s:
            self._fsm.panel_hold_complete()
            self._state_started_s = now_s
            self._hold_started_s = None
            self._arrival.reset()
            self._publish_state()

    def _tick_final_loiter(self, _now_s: float) -> None:
        self._command_xyz = self._target_xyz(self._fsm.route.current)
        self._publish_command_setpoint()
        if self._mode == self._land_mode:
            self._transition(PanelMissionState.WAIT_DISARM)
            return
        if self._mode == self._loiter_mode:
            self._transition(PanelMissionState.AUTO_LAND)
            return
        self._request_mode(self._loiter_mode)

    def _tick_land(self, _now_s: float) -> None:
        if self._mode == self._land_mode:
            self._transition(PanelMissionState.WAIT_DISARM)
            return
        self._request_mode(self._land_mode)

    def _tick_wait_disarm(self, _now_s: float) -> None:
        if self._armed:
            return
        self._transition(PanelMissionState.COMPLETE)
        self._publish_result('SUCCESS')

    def _tick_abort(self) -> None:
        if not self._armed:
            return
        if self._abort_land_requested and self._mode != self._land_mode:
            self._request_mode(self._land_mode)

    def _target_xyz(self, waypoint: RelativeWaypoint):
        if self._launch_xyz is None:
            raise RuntimeError('launch reference unavailable')
        return (
            self._launch_xyz[0] + waypoint.x_m,
            self._launch_xyz[1] + waypoint.y_m,
            self._launch_xyz[2] + self._takeoff_height_m,
        )

    def _advance_command_setpoint(
        self,
        waypoint: RelativeWaypoint,
        now_s: float,
    ) -> None:
        if self._command_xyz is None:
            if self._pose_xyz is None:
                raise RuntimeError('position unavailable for setpoint ramp')
            self._command_xyz = self._pose_xyz
        dt_s = max(0.001, min(0.2, now_s - self._last_setpoint_s))
        self._last_setpoint_s = now_s
        self._command_xyz = advance_position_setpoint(
            self._command_xyz,
            self._target_xyz(waypoint),
            maximum_horizontal_speed_mps=(
                self._maximum_horizontal_setpoint_speed_mps
            ),
            maximum_vertical_speed_mps=(
                self._maximum_vertical_setpoint_speed_mps
            ),
            dt_s=dt_s,
        )

    def _publish_command_setpoint(self) -> None:
        if self._command_xyz is None:
            raise RuntimeError('position command is unavailable')
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = self._command_xyz[0]
        message.pose.position.y = self._command_xyz[1]
        message.pose.position.z = self._command_xyz[2]
        if self._orientation is not None:
            message.pose.orientation = self._orientation
        else:
            message.pose.orientation.w = 1.0
        self._setpoint_publisher.publish(message)

    def _position_error(self, waypoint: RelativeWaypoint) -> float:
        if self._pose_xyz is None:
            return math.inf
        target = self._target_xyz(waypoint)
        return math.dist(self._pose_xyz, target)

    def _speed(self) -> float:
        if self._velocity_xyz is None:
            return math.inf
        return math.sqrt(sum(item * item for item in self._velocity_xyz))

    def _request_mode(self, mode: str) -> None:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self._request(
            f'mode:{mode}', self._mode_client, request, lambda r: r.mode_sent
        )

    def _request_takeoff(self, enabled: bool, action: str) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._request(action, self._takeoff_client, request, lambda r: r.success)

    def _request(
        self,
        action: str,
        client,
        request,
        accepted: Callable,
    ) -> None:
        now_s = time.monotonic()
        if self._pending_action is not None:
            return
        if now_s - self._last_action_s.get(action, -math.inf) < self._action_retry_s:
            return
        if not client.service_is_ready():
            self._last_action_s[action] = now_s
            return
        self._pending_action = action
        self._last_action_s[action] = now_s
        future = client.call_async(request)

        def completed(result) -> None:
            self._pending_action = None
            try:
                if not accepted(result.result()):
                    self.get_logger().warning(f'{action} request rejected')
            except Exception as error:
                self.get_logger().error(f'{action} request failed: {error}')

        future.add_done_callback(completed)

    def _fail(self, reason: str, request_land: bool) -> None:
        self.get_logger().error(reason)
        self._fsm.abort(reason)
        self._abort_land_requested = request_land
        self._publish_state()
        self._publish_result(f'ABORTED: {reason}')
        if self._local_takeoff_enabled:
            self._request_takeoff(False, 'abort_takeoff_disable')
        if request_land and self._armed:
            self._request_mode(self._land_mode)

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self._fsm.state.name))

    def _publish_result(self, result: str) -> None:
        self._result_publisher.publish(String(data=result))


def main(args=None) -> None:
    """Run the independent panel mission node."""
    rclpy.init(args=args)
    node = PanelMissionNode()
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
