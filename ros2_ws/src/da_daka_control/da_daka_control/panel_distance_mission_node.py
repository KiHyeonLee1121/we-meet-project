"""Patrol a fixed route once, then re-walk it holding LiDAR distance."""

# Pass 1 (PATROL): climb to takeoff_height_m, visit each waypoint forward
# (position setpoints, same as panel_mission), pausing patrol_pause_s with no
# distance control. Pass 2 (DISTANCE): from wherever PATROL ended, walk the
# same route backward, handing off to the existing LiDAR distance controller
# to settle at target_distance_m (same as mission_manager) at each waypoint,
# then pause distance_pause_s. Reverse transit stays at target_distance_m
# instead of restoring takeoff_height_m between waypoints. Landing happens at
# the final reverse panel, return to the launch origin at the reverse-transit
# height, and land there. Waypoints are launch-body forward/left offsets. The
# node latches pre-arm ENU position/yaw once and rotates those fixed offsets
# into ENU for the complete mission.

import math
import time
from typing import Callable, Optional

from da_daka_control.mission_manager_node import status_failures
from da_daka_control.panel_distance_mission_fsm import (
    advance_slowed_position_setpoint,
    body_offset_to_enu,
    lidar_referenced_local_z_target,
    MissionPhase,
    PanelDistanceMissionFsm,
    PanelDistanceMissionState,
    StableYawReference,
    wrapped_yaw_error,
)
from da_daka_control.panel_mission_fsm import (
    control_ownership_failures,
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
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger


class PanelDistanceMissionNode(Node):
    """Coordinate arm, takeoff, route, per-panel distance hold, and landing."""

    POSITION_OFFBOARD_STATES = {
        PanelDistanceMissionState.MOVE_TO_PANEL,
        PanelDistanceMissionState.RETURN_HOME,
    }
    VELOCITY_OFFBOARD_STATES = {
        PanelDistanceMissionState.TAKEOFF_HOLD,
        PanelDistanceMissionState.DISTANCE_CONTROL,
        PanelDistanceMissionState.DISTANCE_HOLD,
    }
    ACTIVE_STATES = PanelDistanceMissionFsm.ACTIVE_STATES
    ON_GROUND = 1

    def __init__(self) -> None:
        super().__init__('panel_distance_mission')
        self._declare_parameters()
        self._load_parameters()
        self._fsm = PanelDistanceMissionFsm(self._make_route())
        self._arrival = StableArrival(
            self._arrival_tolerance_m,
            self._arrival_max_speed_mps,
            self._arrival_stable_s,
        )
        self._launch_yaw_stability = StableYawReference(
            self._launch_yaw_stable_duration_s,
            self._launch_yaw_max_deviation_rad,
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, '/panel_distance_mission/state', latched_qos
        )
        self._result_publisher = self.create_publisher(
            String, '/panel_distance_mission/result', latched_qos
        )
        self._setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile_sensor_data,
        )
        self._yaw_target_publisher = self.create_publisher(
            Float32,
            '/vertical_control/yaw_target',
            qos_profile_sensor_data,
        )
        self._horizontal_setpoint_speed_publisher = self.create_publisher(
            Float32,
            '/panel_distance_mission/horizontal_setpoint_speed',
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, '/panel_distance_mission/start', self._start
        )
        self.create_service(
            Trigger, '/panel_distance_mission/abort', self._abort
        )

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
            Range,
            '/distance/filtered',
            self._range,
            qos_profile_sensor_data,
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
            Bool,
            '/distance_control/target_reached',
            self._distance_target_state,
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
        self._distance_enable_client = self.create_client(
            SetBool, '/distance_control/enable'
        )

        self._connected = False
        self._armed = False
        self._mode = ''
        self._landed_state: Optional[int] = None
        self._extended_state_time_s: Optional[float] = None
        self._pose_xyz: Optional[tuple[float, float, float]] = None
        self._orientation = None
        self._current_yaw_rad: Optional[float] = None
        self._current_yaw_time_s: Optional[float] = None
        self._velocity_xyz: Optional[tuple[float, float, float]] = None
        self._pose_time_s: Optional[float] = None
        self._velocity_time_s: Optional[float] = None
        self._battery_remaining: Optional[float] = None
        self._battery_time_s: Optional[float] = None
        self._sensors_enabled: Optional[int] = None
        self._sensors_health: Optional[int] = None
        self._sys_status_time_s: Optional[float] = None
        self._distance_m: Optional[float] = None
        self._range_min_m = 0.0
        self._range_max_m = 0.0
        self._range_time_s: Optional[float] = None
        self._ground_xyz: Optional[tuple[float, float, float]] = None
        self._launch_xyz: Optional[tuple[float, float, float]] = None
        self._launch_yaw_rad: Optional[float] = None
        self._local_takeoff_enabled = False
        self._local_takeoff_reached = False
        self._distance_control_enabled = False
        self._distance_target_reached = False
        self._vertical_control_mode = ''
        self._distance_mission_state = ''
        self._state_started_s = time.monotonic()
        self._prestream_started_s: Optional[float] = None
        self._yaw_aligned_since_s: Optional[float] = None
        self._pause_started_s: Optional[float] = None
        self._distance_control_started_s: Optional[float] = None
        self._distance_hold_started_s: Optional[float] = None
        self._command_xyz: Optional[tuple[float, float, float]] = None
        self._horizontal_setpoint_speed_mps = 0.0
        self._last_setpoint_s = time.monotonic()
        self._pending_action: Optional[str] = None
        self._last_action_s: dict[str, float] = {}
        self._abort_land_requested = False
        self._publish_state()
        self._publish_result('IDLE')
        self.create_timer(1.0 / self._tick_rate_hz, self._tick)
        self.get_logger().info(
            'Panel distance mission ready in IDLE; explicit start service '
            f'required; configuration_approved={self._configuration_approved}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('configuration_approved', False)
        self.declare_parameter('waypoint_forward_m', [0.0])
        self.declare_parameter('waypoint_left_m', [0.0])
        self.declare_parameter('takeoff_height_m', 3.0)
        self.declare_parameter('target_distance_m', 1.0)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('arrival_max_speed_mps', 0.10)
        self.declare_parameter('arrival_stable_s', 2.0)
        self.declare_parameter('maximum_horizontal_setpoint_speed_mps', 0.80)
        self.declare_parameter('horizontal_setpoint_max_accel_mps2', 0.60)
        self.declare_parameter('horizontal_slow_zone_m', 1.20)
        self.declare_parameter('horizontal_min_approach_speed_mps', 0.12)
        self.declare_parameter('horizontal_target_snap_distance_m', 0.05)
        self.declare_parameter('maximum_vertical_setpoint_speed_mps', 0.20)
        self.declare_parameter('cruise_lidar_tolerance_m', 0.10)
        self.declare_parameter('cruise_lidar_control_deadband_m', 0.03)
        self.declare_parameter('cruise_lidar_gain', 1.0)
        self.declare_parameter('cruise_lidar_max_local_z_offset_m', 0.40)
        self.declare_parameter('patrol_pause_s', 1.0)
        self.declare_parameter('distance_pause_s', 3.0)
        self.declare_parameter('prestream_s', 2.0)
        self.declare_parameter('yaw_alignment_tolerance_deg', 5.0)
        self.declare_parameter('yaw_alignment_stable_s', 0.5)
        self.declare_parameter('launch_yaw_stable_duration_s', 1.0)
        self.declare_parameter('launch_yaw_max_deviation_deg', 2.0)
        self.declare_parameter('takeoff_timeout_s', 40.0)
        self.declare_parameter('distance_control_timeout_s', 20.0)
        self.declare_parameter('target_hold_confirm_duration_s', 0.2)
        self.declare_parameter('sensor_timeout_s', 0.3)
        self.declare_parameter('telemetry_timeout_s', 0.5)
        self.declare_parameter('status_timeout_s', 3.0)
        self.declare_parameter('minimum_battery_remaining', 0.15)
        self.declare_parameter('battery_id', 1)
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
        self._waypoint_forward = [
            float(item) for item in value('waypoint_forward_m')
        ]
        self._waypoint_left = [
            float(item) for item in value('waypoint_left_m')
        ]
        self._takeoff_height_m = float(value('takeoff_height_m'))
        self._target_distance_m = float(value('target_distance_m'))
        self._arrival_tolerance_m = float(value('arrival_tolerance_m'))
        self._arrival_max_speed_mps = float(value('arrival_max_speed_mps'))
        self._arrival_stable_s = float(value('arrival_stable_s'))
        self._maximum_horizontal_setpoint_speed_mps = float(
            value('maximum_horizontal_setpoint_speed_mps')
        )
        self._horizontal_setpoint_max_accel_mps2 = float(
            value('horizontal_setpoint_max_accel_mps2')
        )
        self._horizontal_slow_zone_m = float(
            value('horizontal_slow_zone_m')
        )
        self._horizontal_min_approach_speed_mps = float(
            value('horizontal_min_approach_speed_mps')
        )
        self._horizontal_target_snap_distance_m = float(
            value('horizontal_target_snap_distance_m')
        )
        self._maximum_vertical_setpoint_speed_mps = float(
            value('maximum_vertical_setpoint_speed_mps')
        )
        self._cruise_lidar_tolerance_m = float(
            value('cruise_lidar_tolerance_m')
        )
        self._cruise_lidar_control_deadband_m = float(
            value('cruise_lidar_control_deadband_m')
        )
        self._cruise_lidar_gain = float(value('cruise_lidar_gain'))
        self._cruise_lidar_max_offset_m = float(
            value('cruise_lidar_max_local_z_offset_m')
        )
        self._patrol_pause_s = float(value('patrol_pause_s'))
        self._distance_pause_s = float(value('distance_pause_s'))
        self._prestream_s = float(value('prestream_s'))
        self._yaw_alignment_tolerance_rad = math.radians(
            float(value('yaw_alignment_tolerance_deg'))
        )
        self._yaw_alignment_stable_s = float(
            value('yaw_alignment_stable_s')
        )
        self._launch_yaw_stable_duration_s = float(
            value('launch_yaw_stable_duration_s')
        )
        self._launch_yaw_max_deviation_rad = math.radians(
            float(value('launch_yaw_max_deviation_deg'))
        )
        self._takeoff_timeout_s = float(value('takeoff_timeout_s'))
        self._distance_control_timeout_s = float(
            value('distance_control_timeout_s')
        )
        self._target_hold_s = float(value('target_hold_confirm_duration_s'))
        self._sensor_timeout_s = float(value('sensor_timeout_s'))
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
        if len(self._waypoint_forward) != len(self._waypoint_left):
            raise ValueError(
                'waypoint_forward_m and waypoint_left_m lengths differ'
            )
        positive = (
            self._takeoff_height_m,
            self._arrival_tolerance_m,
            self._arrival_stable_s,
            self._maximum_horizontal_setpoint_speed_mps,
            self._horizontal_setpoint_max_accel_mps2,
            self._horizontal_slow_zone_m,
            self._horizontal_min_approach_speed_mps,
            self._horizontal_target_snap_distance_m,
            self._maximum_vertical_setpoint_speed_mps,
            self._cruise_lidar_tolerance_m,
            self._cruise_lidar_control_deadband_m,
            self._cruise_lidar_gain,
            self._cruise_lidar_max_offset_m,
            self._patrol_pause_s,
            self._distance_pause_s,
            self._prestream_s,
            self._yaw_alignment_tolerance_rad,
            self._yaw_alignment_stable_s,
            self._launch_yaw_stable_duration_s,
            self._launch_yaw_max_deviation_rad,
            self._takeoff_timeout_s,
            self._distance_control_timeout_s,
            self._target_hold_s,
            self._sensor_timeout_s,
            self._telemetry_timeout_s,
            self._status_timeout_s,
            self._tick_rate_hz,
            self._action_retry_s,
            self._state_timeout_s,
        )
        if any(item <= 0.0 for item in positive):
            raise ValueError(
                'panel distance mission timing/distance values must be '
                'positive'
            )
        if (
            self._cruise_lidar_control_deadband_m
            > self._cruise_lidar_tolerance_m
        ):
            raise ValueError(
                'cruise LiDAR control deadband cannot exceed tolerance'
            )
        if not 0.0 <= self._minimum_battery <= 1.0:
            raise ValueError('minimum_battery_remaining must be in [0, 1]')
        if not 0 <= self._battery_id <= 9:
            raise ValueError('battery_id must be within [0, 9]')
        if self._ignored_health < 0:
            raise ValueError('ignored_unhealthy_sensor_mask cannot be negative')
        if self._yaw_alignment_tolerance_rad >= math.pi:
            raise ValueError('yaw_alignment_tolerance_deg must be below 180')
        if self._launch_yaw_max_deviation_rad >= math.pi:
            raise ValueError(
                'launch_yaw_max_deviation_deg must be below 180'
            )
        if (
            self._horizontal_min_approach_speed_mps
            > self._maximum_horizontal_setpoint_speed_mps
        ):
            raise ValueError(
                'horizontal minimum approach speed exceeds maximum speed'
            )
        if (
            self._horizontal_target_snap_distance_m
            >= self._horizontal_slow_zone_m
        ):
            raise ValueError(
                'horizontal target snap distance must be below slow zone'
            )

    def _make_route(self) -> PanelRoute:
        return PanelRoute(
            [
                RelativeWaypoint(x_m, y_m)
                for x_m, y_m in zip(
                    self._waypoint_forward,
                    self._waypoint_left,
                )
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
        self._launch_yaw_rad = None
        self._yaw_aligned_since_s = None
        self._launch_yaw_stability.reset()
        self._arrival.reset()
        self._abort_land_requested = False
        self._command_xyz = None
        self._reset_horizontal_setpoint_speed()
        self._distance_control_started_s = None
        self._distance_hold_started_s = None
        self._pause_started_s = None
        self._publish_state()
        self._publish_result('RUNNING')
        response.success = True
        response.message = 'panel distance mission accepted'
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
            quaternion = (
                float(self._orientation.x),
                float(self._orientation.y),
                float(self._orientation.z),
                float(self._orientation.w),
            )
            norm = math.sqrt(sum(item * item for item in quaternion))
            if (
                all(math.isfinite(item) for item in quaternion)
                and norm > 1e-6
            ):
                x, y, z, w = (item / norm for item in quaternion)
                self._current_yaw_rad = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                )
                self._current_yaw_time_s = time.monotonic()
                if self._fsm.state == PanelDistanceMissionState.PRECHECK:
                    self._launch_yaw_stability.update(
                        self._current_yaw_rad,
                        self._current_yaw_time_s,
                    )
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

    def _range(self, message: Range) -> None:
        if not math.isfinite(message.range):
            return
        self._distance_m = float(message.range)
        self._range_min_m = float(message.min_range)
        self._range_max_m = float(message.max_range)
        self._range_time_s = time.monotonic()

    def _takeoff_enabled_state(self, message: Bool) -> None:
        self._local_takeoff_enabled = bool(message.data)

    def _takeoff_target_state(self, message: Bool) -> None:
        self._local_takeoff_reached = bool(message.data)

    def _distance_control_state(self, message: Bool) -> None:
        self._distance_control_enabled = bool(message.data)

    def _distance_target_state(self, message: Bool) -> None:
        self._distance_target_reached = bool(message.data)

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

    def _yaw_fresh(self, now_s: float) -> bool:
        return (
            self._current_yaw_rad is not None
            and self._current_yaw_time_s is not None
            and now_s - self._current_yaw_time_s <= self._telemetry_timeout_s
        )

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
        if not self._yaw_fresh(now_s):
            failures.append('local yaw unavailable or stale')
        if not self._sensor_healthy(now_s):
            failures.append('distance sensor unavailable or invalid')
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

    def _transition(
        self,
        state: PanelDistanceMissionState,
        reason: str = '',
    ) -> None:
        previous = self._fsm.state
        self._fsm.transition(state, reason)
        self._on_fsm_transitioned(previous)

    def _on_fsm_transitioned(
        self,
        previous: PanelDistanceMissionState,
    ) -> None:
        """Apply node-level bookkeeping after any FSM-internal transition."""
        now_s = time.monotonic()
        self._state_started_s = now_s
        self._prestream_started_s = None
        if self._fsm.state == PanelDistanceMissionState.MOVE_OFFBOARD:
            self._yaw_aligned_since_s = None
        if self._fsm.state == PanelDistanceMissionState.RETURN_HOME_OFFBOARD:
            self._yaw_aligned_since_s = None
        if self._fsm.state == PanelDistanceMissionState.PANEL_PAUSE:
            self._pause_started_s = now_s
        self.get_logger().info(
            f'{previous.name} -> {self._fsm.state.name}: {self._fsm.reason}'
        )
        self._publish_state()

    def _tick(self) -> None:
        now_s = time.monotonic()
        state = self._fsm.state
        if state in {
            PanelDistanceMissionState.IDLE,
            PanelDistanceMissionState.COMPLETE,
        }:
            return
        if self._launch_yaw_rad is not None:
            self._yaw_target_publisher.publish(
                Float32(data=self._launch_yaw_rad)
            )
        if state == PanelDistanceMissionState.ABORT:
            self._tick_abort()
            return
        if (
            self._mode == self._land_mode
            and state not in {
                PanelDistanceMissionState.AUTO_LAND,
                PanelDistanceMissionState.WAIT_DISARM,
            }
        ):
            self._fail(
                'external AUTO.LAND confirmed; respecting PX4/QGC',
                request_land=False,
            )
            return
        if (
            state
            not in {
                PanelDistanceMissionState.PRECHECK,
                PanelDistanceMissionState.ARMING,
            }
            and not self._telemetry_fresh(now_s)
        ):
            self._fail('local pose or velocity telemetry timeout', True)
            return
        if (
            state
            not in {
                PanelDistanceMissionState.PRECHECK,
                PanelDistanceMissionState.AUTO_LAND,
                PanelDistanceMissionState.WAIT_DISARM,
            }
            and not self._sensor_healthy(now_s)
        ):
            self._fail('distance sensor timeout or invalid range', True)
            return
        if now_s - self._state_started_s > self._current_state_timeout():
            self._fail(f'{state.name} timeout', request_land=True)
            return
        if state in self.POSITION_OFFBOARD_STATES and self._mode != 'OFFBOARD':
            self._fail(
                f'QGC/PX4 mode override: {self._mode}',
                request_land=False,
            )
            return
        if state in self.VELOCITY_OFFBOARD_STATES and self._mode != 'OFFBOARD':
            self._fail(
                f'QGC/PX4 mode override during distance hold: {self._mode}',
                request_land=False,
            )
            return
        handlers = {
            PanelDistanceMissionState.PRECHECK: self._tick_precheck,
            PanelDistanceMissionState.ARMING: self._tick_arming,
            PanelDistanceMissionState.TAKEOFF: self._tick_takeoff,
            PanelDistanceMissionState.TAKEOFF_PRESTREAM: (
                self._tick_takeoff_prestream
            ),
            PanelDistanceMissionState.TAKEOFF_OFFBOARD: (
                self._tick_takeoff_offboard
            ),
            PanelDistanceMissionState.TAKEOFF_HOLD: self._tick_takeoff_hold,
            PanelDistanceMissionState.LOITER_HANDOVER: (
                self._tick_loiter_handover
            ),
            PanelDistanceMissionState.MOVE_PRESTREAM: (
                self._tick_move_prestream
            ),
            PanelDistanceMissionState.MOVE_OFFBOARD: self._tick_move_offboard,
            PanelDistanceMissionState.MOVE_TO_PANEL: self._tick_move,
            PanelDistanceMissionState.ARRIVE_LOITER: self._tick_arrive_loiter,
            PanelDistanceMissionState.DISTANCE_PRESTREAM: (
                self._tick_distance_prestream
            ),
            PanelDistanceMissionState.DISTANCE_OFFBOARD: (
                self._tick_distance_offboard
            ),
            PanelDistanceMissionState.DISTANCE_CONTROL: (
                self._tick_distance_control
            ),
            PanelDistanceMissionState.DISTANCE_HOLD: self._tick_distance_hold,
            PanelDistanceMissionState.DISTANCE_LOITER: (
                self._tick_distance_loiter
            ),
            PanelDistanceMissionState.PANEL_PAUSE: self._tick_panel_pause,
            PanelDistanceMissionState.RETURN_HOME_PRESTREAM: (
                self._tick_return_home_prestream
            ),
            PanelDistanceMissionState.RETURN_HOME_OFFBOARD: (
                self._tick_return_home_offboard
            ),
            PanelDistanceMissionState.RETURN_HOME: self._tick_return_home,
            PanelDistanceMissionState.FINAL_LOITER: self._tick_final_loiter,
            PanelDistanceMissionState.AUTO_LAND: self._tick_land,
            PanelDistanceMissionState.WAIT_DISARM: self._tick_wait_disarm,
        }
        handler = handlers.get(state)
        if handler is not None:
            handler(now_s)

    def _current_state_timeout(self) -> float:
        if self._fsm.state == PanelDistanceMissionState.TAKEOFF_HOLD:
            return self._takeoff_timeout_s
        if self._fsm.state == PanelDistanceMissionState.DISTANCE_CONTROL:
            return self._distance_control_timeout_s
        return self._state_timeout_s

    # -- precheck / arm / takeoff (identical shape to panel_mission) --------

    def _tick_precheck(self, now_s: float) -> None:
        failures = self._precheck_failures(now_s)
        if failures:
            self._fail('preflight failed: ' + '; '.join(failures), False)
            return
        # A vehicle normally sits in POSCTL before this mission starts.  Move
        # to the mission's known handover mode while it is still disarmed;
        # TAKEOFF_PRESTREAM can then distinguish a real operator override from
        # the expected initial flight mode.
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        if not self._yaw_fresh(now_s):
            self._fail('launch yaw unavailable or stale', False)
            return
        stable_launch_yaw = self._launch_yaw_stability.stable_yaw_rad
        if stable_launch_yaw is None:
            return
        self._launch_xyz = self._ground_xyz
        self._launch_yaw_rad = stable_launch_yaw
        self._yaw_target_publisher.publish(
            Float32(data=self._launch_yaw_rad)
        )
        self.get_logger().info(
            'Latched launch reference: '
            f'ENU=({self._launch_xyz[0]:.3f}, '
            f'{self._launch_xyz[1]:.3f}), '
            f'yaw={math.degrees(self._launch_yaw_rad):.2f} deg'
        )
        route_text = []
        for index, (forward_m, left_m) in enumerate(
            zip(self._waypoint_forward, self._waypoint_left),
            1,
        ):
            east_m, north_m = body_offset_to_enu(
                forward_m=forward_m,
                left_m=left_m,
                launch_yaw_rad=self._launch_yaw_rad,
            )
            route_text.append(
                f'{index}=({self._launch_xyz[0] + east_m:.3f}, '
                f'{self._launch_xyz[1] + north_m:.3f})'
            )
        self.get_logger().info(
            'Latched ENU route: ' + ', '.join(route_text)
        )
        self._transition(PanelDistanceMissionState.ARMING)

    def _tick_arming(self, _now_s: float) -> None:
        if self._armed:
            self._transition(PanelDistanceMissionState.TAKEOFF)
            return
        request = CommandBool.Request()
        request.value = True
        self._request('arm', self._arm_client, request, lambda r: r.success)

    def _tick_takeoff(self, _now_s: float) -> None:
        if self._local_takeoff_enabled:
            self._transition(PanelDistanceMissionState.TAKEOFF_PRESTREAM)
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
            self._transition(PanelDistanceMissionState.TAKEOFF_OFFBOARD)

    def _tick_takeoff_offboard(self, _now_s: float) -> None:
        if self._mode == 'OFFBOARD':
            self._transition(PanelDistanceMissionState.TAKEOFF_HOLD)
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
            self._transition(PanelDistanceMissionState.LOITER_HANDOVER)

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
        self._reset_horizontal_setpoint_speed()
        self._last_setpoint_s = time.monotonic()
        self._transition(PanelDistanceMissionState.MOVE_PRESTREAM)

    # -- move to waypoint (identical shape to panel_mission) -----------------

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
            self._transition(PanelDistanceMissionState.MOVE_OFFBOARD)

    def _tick_move_offboard(self, _now_s: float) -> None:
        self._publish_command_setpoint()
        if self._mode == 'OFFBOARD':
            if not self._yaw_alignment_ready(time.monotonic()):
                return
            self._arrival.reset()
            self._transition(PanelDistanceMissionState.MOVE_TO_PANEL)
            return
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before movement OFFBOARD: {self._mode}',
                request_land=False,
            )
            return
        self._request_mode('OFFBOARD')

    def _yaw_alignment_ready(self, now_s: float) -> bool:
        if self._launch_yaw_rad is None or not self._yaw_fresh(now_s):
            self._fail('launch or current yaw unavailable', True)
            return False
        yaw_error_rad = abs(
            wrapped_yaw_error(
                self._launch_yaw_rad,
                self._current_yaw_rad,
            )
        )
        if yaw_error_rad > self._yaw_alignment_tolerance_rad:
            self._yaw_aligned_since_s = None
            return False
        if self._yaw_aligned_since_s is None:
            self._yaw_aligned_since_s = now_s
            return False
        return (
            now_s - self._yaw_aligned_since_s
            >= self._yaw_alignment_stable_s
        )

    def _move_target_distance_m(self) -> float:
        # Reverse-pass transit holds the panel distance-hold altitude
        # instead of climbing back to cruise height between waypoints.
        if self._fsm.phase is MissionPhase.DISTANCE:
            return self._target_distance_m
        return self._takeoff_height_m

    def _tick_move(self, now_s: float) -> None:
        target = self._fsm.route.current
        self._advance_command_setpoint(target, now_s)
        self._publish_command_setpoint()
        horizontal_error = self._horizontal_position_error(target)
        horizontal_speed = self._horizontal_speed()
        lidar_height_ready = (
            self._distance_m is not None
            and abs(self._move_target_distance_m() - self._distance_m)
            <= self._cruise_lidar_tolerance_m
        )
        if self._arrival.update(
            position_error_m=horizontal_error,
            speed_mps=horizontal_speed,
            now_s=now_s,
            telemetry_valid=(
                self._telemetry_fresh(now_s)
                and self._sensor_healthy(now_s)
                and lidar_height_ready
            ),
        ):
            previous = self._fsm.state
            self._fsm.panel_move_arrived()
            self._on_fsm_transitioned(previous)

    # -- per-panel distance hold (identical shape to mission_manager) -------

    def _tick_arrive_loiter(self, _now_s: float) -> None:
        # Leave OFFBOARD before the distance controller becomes the sole
        # vertical setpoint owner, same handover discipline as mission_manager
        # uses between local-takeoff and LiDAR distance control.
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        self._transition(PanelDistanceMissionState.DISTANCE_PRESTREAM)

    def _tick_distance_prestream(self, now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before distance hold: {self._mode}',
                request_land=False,
            )
            return
        if not self._distance_control_enabled:
            self._request_distance_enable(True, 'distance_enable')
            return
        if self._prestream_started_s is None:
            self._prestream_started_s = now_s
        if now_s - self._prestream_started_s >= self._prestream_s:
            self._transition(PanelDistanceMissionState.DISTANCE_OFFBOARD)

    def _tick_distance_offboard(self, _now_s: float) -> None:
        if self._mode == 'OFFBOARD':
            self._distance_control_started_s = time.monotonic()
            self._transition(PanelDistanceMissionState.DISTANCE_CONTROL)
            return
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before distance OFFBOARD: {self._mode}',
                request_land=False,
            )
            return
        self._request_mode('OFFBOARD')

    def _tick_distance_control(self, now_s: float) -> None:
        if not self._distance_control_enabled:
            self._fail('distance controller unexpectedly disabled', True)
            return
        if self._distance_target_reached:
            self._transition(PanelDistanceMissionState.DISTANCE_HOLD)

    def _tick_distance_hold(self, now_s: float) -> None:
        if not self._distance_target_reached:
            self._distance_hold_started_s = None
            self._transition(PanelDistanceMissionState.DISTANCE_CONTROL)
            return
        if self._distance_hold_started_s is None:
            self._distance_hold_started_s = now_s
        if now_s - self._distance_hold_started_s >= self._target_hold_s:
            self._transition(PanelDistanceMissionState.DISTANCE_LOITER)

    def _tick_distance_loiter(self, _now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        if self._distance_control_enabled:
            self._request_distance_enable(False, 'distance_disable')
            return
        previous = self._fsm.state
        self._fsm.distance_hold_complete()
        self._on_fsm_transitioned(previous)

    def _current_pause_s(self) -> float:
        if self._fsm.phase == MissionPhase.PATROL:
            return self._patrol_pause_s
        return self._distance_pause_s

    def _tick_panel_pause(self, now_s: float) -> None:
        if self._fsm.phase == MissionPhase.PATROL:
            self._advance_command_setpoint(self._fsm.route.current, now_s)
            self._publish_command_setpoint()
        if self._pause_started_s is None:
            self._pause_started_s = now_s
        if now_s - self._pause_started_s < self._current_pause_s():
            return
        self._distance_control_started_s = None
        self._distance_hold_started_s = None
        self._pause_started_s = None
        if self._pose_xyz is not None:
            self._command_xyz = self._pose_xyz
            self._reset_horizontal_setpoint_speed()
            self._last_setpoint_s = time.monotonic()
        previous = self._fsm.state
        self._fsm.panel_pause_complete()
        self._on_fsm_transitioned(previous)
        self._arrival.reset()

    # -- final return to the launch origin ----------------------------------

    def _tick_return_home_prestream(self, now_s: float) -> None:
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before return home: {self._mode}',
                request_land=False,
            )
            return
        self._publish_command_setpoint()
        if self._prestream_started_s is None:
            self._prestream_started_s = now_s
        if now_s - self._prestream_started_s >= self._prestream_s:
            self._transition(
                PanelDistanceMissionState.RETURN_HOME_OFFBOARD
            )

    def _tick_return_home_offboard(self, _now_s: float) -> None:
        self._publish_command_setpoint()
        if self._mode == 'OFFBOARD':
            if not self._yaw_alignment_ready(time.monotonic()):
                return
            self._arrival.reset()
            self._transition(PanelDistanceMissionState.RETURN_HOME)
            return
        if self._mode != self._loiter_mode:
            self._fail(
                f'QGC/PX4 mode override before return OFFBOARD: {self._mode}',
                request_land=False,
            )
            return
        self._request_mode('OFFBOARD')

    def _tick_return_home(self, now_s: float) -> None:
        home = RelativeWaypoint(0.0, 0.0)
        self._advance_command_setpoint(home, now_s)
        self._publish_command_setpoint()
        lidar_height_ready = (
            self._distance_m is not None
            and abs(self._target_distance_m - self._distance_m)
            <= self._cruise_lidar_tolerance_m
        )
        if self._arrival.update(
            position_error_m=self._horizontal_position_error(home),
            speed_mps=self._horizontal_speed(),
            now_s=now_s,
            telemetry_valid=(
                self._telemetry_fresh(now_s)
                and self._sensor_healthy(now_s)
                and lidar_height_ready
            ),
        ):
            self._transition(PanelDistanceMissionState.FINAL_LOITER)

    # -- landing (identical shape to panel_mission) --------------------------

    def _tick_final_loiter(self, _now_s: float) -> None:
        if self._mode == self._land_mode:
            self._transition(PanelDistanceMissionState.WAIT_DISARM)
            return
        if self._mode == self._loiter_mode:
            self._transition(PanelDistanceMissionState.AUTO_LAND)
            return
        self._request_mode(self._loiter_mode)

    def _tick_land(self, _now_s: float) -> None:
        if self._mode == self._land_mode:
            self._transition(PanelDistanceMissionState.WAIT_DISARM)
            return
        self._request_mode(self._land_mode)

    def _tick_wait_disarm(self, _now_s: float) -> None:
        if self._armed:
            return
        self._transition(PanelDistanceMissionState.COMPLETE)
        self._publish_result('SUCCESS')

    def _tick_abort(self) -> None:
        if self._distance_control_enabled:
            self._request_distance_enable(False, 'abort_distance_disable')
            return
        if self._local_takeoff_enabled:
            self._request_takeoff(False, 'abort_takeoff_disable')
            return
        if not self._armed:
            return
        if self._abort_land_requested and self._mode != self._land_mode:
            self._request_mode(self._land_mode)

    # -- setpoint helpers (identical shape to panel_mission) ------------------

    def _reset_horizontal_setpoint_speed(self) -> None:
        """Reset and record the moving horizontal target speed."""
        self._horizontal_setpoint_speed_mps = 0.0
        self._horizontal_setpoint_speed_publisher.publish(Float32(data=0.0))

    def _target_xyz(self, waypoint: RelativeWaypoint):
        if self._launch_xyz is None or self._pose_xyz is None:
            raise RuntimeError('launch or local position unavailable')
        if self._launch_yaw_rad is None:
            raise RuntimeError('launch yaw unavailable')
        if self._distance_m is None:
            raise RuntimeError('LiDAR distance unavailable')
        east_m, north_m = body_offset_to_enu(
            forward_m=waypoint.x_m,
            left_m=waypoint.y_m,
            launch_yaw_rad=self._launch_yaw_rad,
        )
        lidar_target_z = lidar_referenced_local_z_target(
            local_z_m=self._pose_xyz[2],
            measured_distance_m=self._distance_m,
            target_distance_m=self._move_target_distance_m(),
            gain=self._cruise_lidar_gain,
            maximum_offset_m=self._cruise_lidar_max_offset_m,
            tolerance_m=self._cruise_lidar_control_deadband_m,
        )
        return (
            self._launch_xyz[0] + east_m,
            self._launch_xyz[1] + north_m,
            lidar_target_z,
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
            self._reset_horizontal_setpoint_speed()
        dt_s = max(0.001, min(0.2, now_s - self._last_setpoint_s))
        self._last_setpoint_s = now_s
        self._command_xyz, self._horizontal_setpoint_speed_mps = (
            advance_slowed_position_setpoint(
                self._command_xyz,
                self._target_xyz(waypoint),
                (self._pose_xyz[0], self._pose_xyz[1]),
                current_horizontal_speed_mps=(
                    self._horizontal_setpoint_speed_mps
                ),
                maximum_horizontal_speed_mps=(
                    self._maximum_horizontal_setpoint_speed_mps
                ),
                maximum_horizontal_accel_mps2=(
                    self._horizontal_setpoint_max_accel_mps2
                ),
                horizontal_slow_zone_m=self._horizontal_slow_zone_m,
                minimum_approach_speed_mps=(
                    self._horizontal_min_approach_speed_mps
                ),
                target_snap_distance_m=(
                    self._horizontal_target_snap_distance_m
                ),
                maximum_vertical_speed_mps=(
                    self._maximum_vertical_setpoint_speed_mps
                ),
                dt_s=dt_s,
            )
        )
        self._horizontal_setpoint_speed_publisher.publish(
            Float32(data=self._horizontal_setpoint_speed_mps)
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
        if self._launch_yaw_rad is None:
            raise RuntimeError('launch yaw unavailable for position setpoint')
        half_yaw = 0.5 * self._launch_yaw_rad
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        self._setpoint_publisher.publish(message)

    def _horizontal_position_error(
        self,
        waypoint: RelativeWaypoint,
    ) -> float:
        if self._pose_xyz is None:
            return math.inf
        if self._launch_xyz is None:
            return math.inf
        if self._launch_yaw_rad is None:
            return math.inf
        east_m, north_m = body_offset_to_enu(
            forward_m=waypoint.x_m,
            left_m=waypoint.y_m,
            launch_yaw_rad=self._launch_yaw_rad,
        )
        target_x = self._launch_xyz[0] + east_m
        target_y = self._launch_xyz[1] + north_m
        return math.hypot(
            self._pose_xyz[0] - target_x,
            self._pose_xyz[1] - target_y,
        )

    def _horizontal_speed(self) -> float:
        if self._velocity_xyz is None:
            return math.inf
        return math.hypot(self._velocity_xyz[0], self._velocity_xyz[1])

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

    def _request_distance_enable(self, enabled: bool, action: str) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._request(
            action,
            self._distance_enable_client,
            request,
            lambda r: r.success,
        )

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

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self._fsm.state.name))

    def _publish_result(self, result: str) -> None:
        self._result_publisher.publish(String(data=result))


def main(args=None) -> None:
    """Run the panel-route + distance-hold mission node."""
    rclpy.init(args=args)
    node = PanelDistanceMissionNode()
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
