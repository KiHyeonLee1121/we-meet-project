"""Run the complete Pi-owned survey, clean, verify and return mission."""

import json
import math
import time
from typing import Callable, Optional

from da_daka_control.autonomous_cleaning_fsm import (
    AutonomousCleaningFsm,
    CleaningMissionState,
)
from da_daka_control.mission_manager_node import (
    inflight_battery_failure,
    sag_compensated_remaining,
    StableWindow,
    status_failures,
)
from da_daka_control.panel_distance_mission_fsm import (
    advance_slowed_position_setpoint,
    early_takeoff_constant_position_allowed,
    horizontal_estimator_failures,
    lidar_referenced_local_z_target,
    StableYawReference,
    TimeWindowMedian,
    wrapped_yaw_error,
)
from da_daka_control.panel_mapping import PanelTarget
from da_daka_control.panel_mission_fsm import StableArrival
from da_daka_control.route_planner import plan_panel_route
from da_daka_control.spray_sequence import (
    perception_is_newer,
    PerceptionBarrier,
    SprayCycleTracker,
)
from da_daka_control.survey_planner import (
    rectangular_scan_waypoints,
    survey_timeout_elapsed_s,
)
from da_daka_interfaces.msg import PanelMap, PerceptionResult
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger


class AutonomousCleaningMissionNode(Node):
    """Own MAVROS setpoints and coordinate every panel from one Pi FSM."""

    ON_GROUND = 1
    POSITION_STATES = {
        CleaningMissionState.SURVEY,
        CleaningMissionState.TRANSIT,
        CleaningMissionState.SLOW_APPROACH,
        CleaningMissionState.REACQUIRE,
        CleaningMissionState.ASSESS,
        CleaningMissionState.RETURN_HOME,
    }
    VELOCITY_STATES = {
        CleaningMissionState.TAKEOFF,
        CleaningMissionState.DESCEND,
        CleaningMissionState.PRECISION_ALIGN,
        CleaningMissionState.SPRAY,
        CleaningMissionState.POST_SPRAY_ALIGN,
        CleaningMissionState.VERIFY,
    }

    def __init__(self) -> None:
        super().__init__('autonomous_cleaning_mission')
        self._declare_parameters()
        self._load_parameters()
        self._fsm = AutonomousCleaningFsm(self._max_spray_attempts)
        self._arrival = StableArrival(
            self._arrival_tolerance_m,
            self._arrival_max_speed_mps,
            self._arrival_stable_s,
        )
        self._launch_yaw_stability = StableYawReference(
            self._launch_yaw_stable_s,
            self._launch_yaw_max_deviation_rad,
        )
        self._perception_window = StableWindow(self._perception_stable_s)
        self._alignment_window = StableWindow(self._alignment_stable_s)
        self._survey_home_window = StableWindow(self._survey_home_stable_s)
        self._survey_home_speed_filter = TimeWindowMedian(
            self._survey_home_speed_filter_window_s
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, self._state_topic, latched_qos
        )
        self._result_publisher = self.create_publisher(
            String, self._result_topic, latched_qos
        )
        self._readiness_publisher = self.create_publisher(
            String, self._readiness_topic, latched_qos
        )
        self._survey_publisher = self.create_publisher(
            Bool, '/panel_survey/active', latched_qos
        )
        self._ai_mode_publisher = self.create_publisher(
            String, self._ai_mode_topic, latched_qos
        )
        self._panel_id_publisher = self.create_publisher(
            Int32, self._panel_id_topic, latched_qos
        )
        self._yaw_target_publisher = self.create_publisher(
            Float32, '/vertical_control/yaw_target', qos_profile_sensor_data
        )
        self._position_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile_sensor_data,
        )
        self._velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, self._start_service, self._start_callback
        )
        self.create_service(
            Trigger, self._abort_service, self._abort_callback
        )

        self._create_subscriptions(latched_qos)
        self._arm_client = self.create_client(
            CommandBool, '/mavros/cmd/arming'
        )
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._takeoff_client = self.create_client(
            SetBool, '/local_takeoff/enable'
        )
        self._distance_client = self.create_client(
            SetBool, '/distance_control/enable'
        )
        self._spray_enable_client = self.create_client(
            SetBool, '/spray/enable'
        )
        self._spray_trigger_client = self.create_client(
            Trigger, '/spray/trigger'
        )
        self._spray_stop_client = self.create_client(
            Trigger, '/spray/stop'
        )

        self._initialize_runtime_state()
        self._publish_state()
        self._publish_result('IDLE')
        self._publish_survey(False)
        self._publish_ai_mode('idle')
        self.create_timer(1.0 / self._tick_rate_hz, self._tick)
        self.create_timer(0.5, self._publish_readiness)
        self.get_logger().info(
            'Autonomous cleaning mission ready; explicit start required; '
            f'configuration_approved={self._configuration_approved}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('localization_test_mode', False)
        self.declare_parameter('required_survey_panels', 0)
        self.declare_parameter('localization_position_tolerance_m', 0.25)
        self.declare_parameter('localization_scan_width_m', 3.0)
        self.declare_parameter('localization_scan_depth_m', 2.0)
        self.declare_parameter('localization_scan_yaw_aligned', False)
        self.declare_parameter('localization_scan_forward_offset_m', 0.0)
        self.declare_parameter('localization_scan_lateral_offset_m', 0.0)
        self.declare_parameter('state_topic', '/autonomous_cleaning/state')
        self.declare_parameter('result_topic', '/autonomous_cleaning/result')
        self.declare_parameter(
            'readiness_topic', '/autonomous_cleaning/readiness'
        )
        self.declare_parameter('ai_mode_topic', '/ai/requested_mode')
        self.declare_parameter(
            'panel_id_topic', '/autonomous_cleaning/current_panel_id'
        )
        self.declare_parameter(
            'start_service', '/autonomous_cleaning/start'
        )
        self.declare_parameter(
            'abort_service', '/autonomous_cleaning/abort'
        )
        self.declare_parameter('configuration_approved', False)
        self.declare_parameter('calibration_approved', False)
        self.declare_parameter('require_live_spray', True)
        self.declare_parameter('survey_duration_s', 3.0)
        self.declare_parameter('survey_distance_m', 3.0)
        self.declare_parameter('survey_home_tolerance_m', 0.15)
        self.declare_parameter('survey_home_max_speed_mps', 0.05)
        self.declare_parameter('survey_home_stable_s', 2.0)
        self.declare_parameter('survey_home_speed_filter_window_s', 0.30)
        self.declare_parameter('survey_home_yaw_tolerance_deg', 3.0)
        self.declare_parameter('spray_distance_m', 1.0)
        self.declare_parameter('spray_duration_s', 3.0)
        self.declare_parameter('maximum_survey_panels', 32)
        self.declare_parameter('max_spray_attempts', 3)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('arrival_max_speed_mps', 0.10)
        self.declare_parameter('arrival_stable_s', 1.0)
        self.declare_parameter('cruise_speed_mps', 0.65)
        self.declare_parameter('panel_visible_speed_mps', 0.18)
        self.declare_parameter('horizontal_accel_mps2', 0.50)
        self.declare_parameter('horizontal_slow_zone_m', 1.0)
        self.declare_parameter('minimum_approach_speed_mps', 0.10)
        self.declare_parameter('target_snap_distance_m', 0.10)
        self.declare_parameter('maximum_vertical_setpoint_speed_mps', 0.20)
        self.declare_parameter('lidar_z_gain', 1.0)
        self.declare_parameter('lidar_z_tolerance_m', 0.10)
        self.declare_parameter('lidar_z_control_deadband_m', 0.03)
        self.declare_parameter('lidar_z_max_offset_m', 0.35)
        self.declare_parameter('reacquire_radius_m', 0.25)
        self.declare_parameter('reacquire_period_s', 5.0)
        self.declare_parameter('prestream_s', 2.0)
        self.declare_parameter('perception_stable_s', 0.5)
        self.declare_parameter('alignment_stable_s', 0.7)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('launch_yaw_stable_s', 1.0)
        self.declare_parameter('launch_yaw_max_deviation_deg', 2.0)
        self.declare_parameter('takeoff_const_pos_grace_s', 2.0)
        self.declare_parameter('sensor_timeout_s', 0.4)
        self.declare_parameter('telemetry_timeout_s', 0.5)
        self.declare_parameter('perception_timeout_s', 0.7)
        self.declare_parameter('status_timeout_s', 3.0)
        self.declare_parameter('state_timeout_s', 40.0)
        self.declare_parameter('takeoff_timeout_s', 45.0)
        self.declare_parameter('survey_timeout_s', 40.0)
        self.declare_parameter('reacquire_timeout_s', 20.0)
        self.declare_parameter('verification_timeout_s', 20.0)
        self.declare_parameter('minimum_battery_remaining', 0.10)
        # PX4 reports charge off the sagging terminal voltage, so hover
        # current alone can fake a depleted pack. Undo the resistive drop
        # before gating, and require the drop to persist.
        self.declare_parameter('battery_cell_count', 4)
        self.declare_parameter('battery_internal_resistance_ohm', 0.04)
        self.declare_parameter('battery_empty_volts_per_cell', 3.50)
        self.declare_parameter('battery_full_volts_per_cell', 4.05)
        self.declare_parameter('battery_sustained_below_s', 3.0)
        self.declare_parameter('require_enabled_sensors_healthy', True)
        self.declare_parameter('ignored_unhealthy_sensor_mask', 0)
        self.declare_parameter('loiter_mode', 'AUTO.LOITER')
        self.declare_parameter('land_mode', 'AUTO.LAND')
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('action_retry_s', 1.0)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._localization_test_mode = bool(value('localization_test_mode'))
        self._required_survey_panels = int(value('required_survey_panels'))
        self._localization_position_tolerance_m = float(
            value('localization_position_tolerance_m')
        )
        self._localization_scan_width_m = float(
            value('localization_scan_width_m')
        )
        self._localization_scan_depth_m = float(
            value('localization_scan_depth_m')
        )
        self._localization_scan_yaw_aligned = bool(
            value('localization_scan_yaw_aligned')
        )
        self._localization_scan_forward_offset_m = float(
            value('localization_scan_forward_offset_m')
        )
        self._localization_scan_lateral_offset_m = float(
            value('localization_scan_lateral_offset_m')
        )
        self._state_topic = str(value('state_topic'))
        self._result_topic = str(value('result_topic'))
        self._readiness_topic = str(value('readiness_topic'))
        self._ai_mode_topic = str(value('ai_mode_topic'))
        self._panel_id_topic = str(value('panel_id_topic'))
        self._start_service = str(value('start_service'))
        self._abort_service = str(value('abort_service'))
        self._configuration_approved = bool(value('configuration_approved'))
        self._calibration_approved = bool(value('calibration_approved'))
        self._require_live_spray = bool(value('require_live_spray'))
        self._survey_duration_s = float(value('survey_duration_s'))
        self._survey_distance_m = float(value('survey_distance_m'))
        self._survey_home_tolerance_m = float(
            value('survey_home_tolerance_m')
        )
        self._survey_home_max_speed_mps = float(
            value('survey_home_max_speed_mps')
        )
        self._survey_home_stable_s = float(value('survey_home_stable_s'))
        self._survey_home_speed_filter_window_s = float(
            value('survey_home_speed_filter_window_s')
        )
        self._survey_home_yaw_tolerance_rad = math.radians(
            float(value('survey_home_yaw_tolerance_deg'))
        )
        self._spray_distance_m = float(value('spray_distance_m'))
        self._spray_duration_s = float(value('spray_duration_s'))
        self._maximum_survey_panels = int(value('maximum_survey_panels'))
        self._max_spray_attempts = int(value('max_spray_attempts'))
        self._arrival_tolerance_m = float(value('arrival_tolerance_m'))
        self._arrival_max_speed_mps = float(value('arrival_max_speed_mps'))
        self._arrival_stable_s = float(value('arrival_stable_s'))
        self._cruise_speed_mps = float(value('cruise_speed_mps'))
        self._visible_speed_mps = float(value('panel_visible_speed_mps'))
        self._horizontal_accel_mps2 = float(value('horizontal_accel_mps2'))
        self._horizontal_slow_zone_m = float(value('horizontal_slow_zone_m'))
        self._minimum_approach_speed_mps = float(
            value('minimum_approach_speed_mps')
        )
        self._target_snap_distance_m = float(value('target_snap_distance_m'))
        self._maximum_vertical_speed_mps = float(
            value('maximum_vertical_setpoint_speed_mps')
        )
        self._lidar_z_gain = float(value('lidar_z_gain'))
        self._lidar_z_tolerance_m = float(value('lidar_z_tolerance_m'))
        self._lidar_z_control_deadband_m = float(
            value('lidar_z_control_deadband_m')
        )
        self._lidar_z_max_offset_m = float(value('lidar_z_max_offset_m'))
        self._reacquire_radius_m = float(value('reacquire_radius_m'))
        self._reacquire_period_s = float(value('reacquire_period_s'))
        self._prestream_s = float(value('prestream_s'))
        self._perception_stable_s = float(value('perception_stable_s'))
        self._alignment_stable_s = float(value('alignment_stable_s'))
        self._yaw_tolerance_rad = math.radians(float(value('yaw_tolerance_deg')))
        self._launch_yaw_stable_s = float(value('launch_yaw_stable_s'))
        self._launch_yaw_max_deviation_rad = math.radians(
            float(value('launch_yaw_max_deviation_deg'))
        )
        self._takeoff_const_pos_grace_s = float(
            value('takeoff_const_pos_grace_s')
        )
        self._sensor_timeout_s = float(value('sensor_timeout_s'))
        self._telemetry_timeout_s = float(value('telemetry_timeout_s'))
        self._perception_timeout_s = float(value('perception_timeout_s'))
        self._status_timeout_s = float(value('status_timeout_s'))
        self._state_timeout_s = float(value('state_timeout_s'))
        self._takeoff_timeout_s = float(value('takeoff_timeout_s'))
        self._survey_timeout_s = float(value('survey_timeout_s'))
        self._reacquire_timeout_s = float(value('reacquire_timeout_s'))
        self._verification_timeout_s = float(value('verification_timeout_s'))
        self._minimum_battery = float(value('minimum_battery_remaining'))
        self._battery_cell_count = int(value('battery_cell_count'))
        self._battery_internal_resistance_ohm = float(
            value('battery_internal_resistance_ohm')
        )
        self._battery_empty_volts_per_cell = float(
            value('battery_empty_volts_per_cell')
        )
        self._battery_full_volts_per_cell = float(
            value('battery_full_volts_per_cell')
        )
        self._battery_sustained_below_s = float(
            value('battery_sustained_below_s')
        )
        if self._battery_cell_count <= 0:
            raise ValueError('battery_cell_count must be positive')
        if self._battery_internal_resistance_ohm < 0.0:
            raise ValueError('battery internal resistance must not be negative')
        if not (
            self._battery_empty_volts_per_cell
            < self._battery_full_volts_per_cell
        ):
            raise ValueError('battery empty voltage must be below full voltage')
        if self._battery_sustained_below_s < 0.0:
            raise ValueError('battery_sustained_below_s must not be negative')
        self._require_health = bool(value('require_enabled_sensors_healthy'))
        self._ignored_health = int(value('ignored_unhealthy_sensor_mask'))
        self._loiter_mode = str(value('loiter_mode'))
        self._land_mode = str(value('land_mode'))
        self._tick_rate_hz = float(value('tick_rate_hz'))
        self._action_retry_s = float(value('action_retry_s'))
        positive = (
            self._survey_duration_s,
            self._survey_distance_m,
            self._survey_home_tolerance_m,
            self._survey_home_max_speed_mps,
            self._survey_home_stable_s,
            self._survey_home_speed_filter_window_s,
            self._survey_home_yaw_tolerance_rad,
            self._spray_distance_m,
            self._spray_duration_s,
            self._arrival_tolerance_m,
            self._arrival_max_speed_mps,
            self._arrival_stable_s,
            self._cruise_speed_mps,
            self._visible_speed_mps,
            self._horizontal_accel_mps2,
            self._horizontal_slow_zone_m,
            self._minimum_approach_speed_mps,
            self._target_snap_distance_m,
            self._maximum_vertical_speed_mps,
            self._lidar_z_gain,
            self._lidar_z_tolerance_m,
            self._lidar_z_control_deadband_m,
            self._lidar_z_max_offset_m,
            self._reacquire_period_s,
            self._prestream_s,
            self._takeoff_const_pos_grace_s,
            self._tick_rate_hz,
        )
        if any(item <= 0.0 for item in positive):
            raise ValueError('mission motion/timing parameters must be positive')
        if self._visible_speed_mps > self._cruise_speed_mps:
            raise ValueError('panel-visible speed must not exceed cruise speed')
        if self._lidar_z_control_deadband_m > self._lidar_z_tolerance_m:
            raise ValueError(
                'LiDAR control deadband cannot exceed acceptance tolerance'
            )
        if (
            self._survey_home_speed_filter_window_s
            > self._telemetry_timeout_s
        ):
            raise ValueError(
                'survey-home speed filter cannot exceed telemetry timeout'
            )
        if self._maximum_survey_panels <= 0 or self._max_spray_attempts <= 0:
            raise ValueError('panel/attempt limits must be positive')
        if self._required_survey_panels < 0:
            raise ValueError('required_survey_panels cannot be negative')
        if self._required_survey_panels > self._maximum_survey_panels:
            raise ValueError(
                'required_survey_panels cannot exceed maximum_survey_panels'
            )
        if (
            not math.isfinite(self._localization_position_tolerance_m)
            or self._localization_position_tolerance_m <= 0.0
        ):
            raise ValueError('localization position tolerance must be positive')
        if (
            not math.isfinite(self._localization_scan_width_m)
            or not math.isfinite(self._localization_scan_depth_m)
            or self._localization_scan_width_m <= 0.0
            or self._localization_scan_depth_m <= 0.0
        ):
            raise ValueError('localization scan dimensions must be positive')
        if self._max_spray_attempts != 3:
            raise ValueError('max_spray_attempts must be exactly 3')
        if not math.isclose(self._spray_duration_s, 3.0, abs_tol=1e-9):
            raise ValueError('spray_duration_s must be exactly 3.0 seconds')

    def _create_subscriptions(self, latched_qos) -> None:
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(
            ExtendedState, '/mavros/extended_state', self._extended_cb, 10
        )
        self.create_subscription(
            EstimatorStatus,
            '/mavros/estimator_status',
            self._estimator_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(SysStatus, '/mavros/sys_status', self._sys_cb, 10)
        self.create_subscription(
            Range, '/distance/filtered', self._range_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PanelMap, '/panel_survey/map', self._panel_map_cb, latched_qos
        )
        self.create_subscription(
            PerceptionResult,
            '/ai/perception',
            self._perception_cb,
            latched_qos,
        )
        self.create_subscription(Bool, '/ai/health', self._ai_health_cb, latched_qos)
        self.create_subscription(
            TwistStamped,
            '/distance_control/cmd_vel_internal',
            self._distance_command_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            '/visual_servo/cmd_vel_xy',
            self._visual_command_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            '/visual_servo/aligned',
            self._visual_aligned_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/visual_servo/target_valid',
            self._visual_valid_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/visual_servo/panel_visible',
            self._panel_visible_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/distance_control/enabled',
            self._distance_enabled_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/distance_control/target_reached',
            self._distance_reached_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/enabled',
            self._takeoff_enabled_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/local_takeoff/target_reached',
            self._takeoff_reached_cb,
            latched_qos,
        )
        self.create_subscription(
            String,
            '/vertical_control/mode',
            self._vertical_mode_cb,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/altitude_guard/triggered',
            self._altitude_guard_cb,
            latched_qos,
        )
        self.create_subscription(
            String, '/spray/status', self._spray_status_cb, 10
        )
        self.create_subscription(
            String, '/mission/state', self._legacy_mission_cb, latched_qos
        )
        self.create_subscription(
            String,
            '/panel_distance_mission/state',
            self._panel_mission_cb,
            latched_qos,
        )

    def _initialize_runtime_state(self) -> None:
        now_s = time.monotonic()
        self._connected = False
        self._armed = False
        self._mode = ''
        self._landed_state: Optional[int] = None
        self._extended_time_s: Optional[float] = None
        self._estimator_time_s: Optional[float] = None
        self._estimator_attitude_valid: Optional[bool] = None
        self._estimator_horizontal_velocity_valid: Optional[bool] = None
        self._estimator_horizontal_relative_position_valid: Optional[bool] = None
        self._estimator_horizontal_absolute_position_valid: Optional[bool] = None
        self._estimator_constant_position_mode: Optional[bool] = None
        self._pose_xyz: Optional[tuple[float, float, float]] = None
        self._yaw_rad: Optional[float] = None
        self._pose_time_s: Optional[float] = None
        self._velocity_xyz: Optional[tuple[float, float, float]] = None
        self._velocity_time_s: Optional[float] = None
        self._battery_remaining: Optional[float] = None
        self._battery_time_s: Optional[float] = None
        self._battery_reported_remaining: Optional[float] = None
        self._battery_voltage_v: Optional[float] = None
        self._battery_current_a: Optional[float] = None
        self._battery_below_since_s: Optional[float] = None
        self._survey_diagnostics: dict = {}
        self._sensors_enabled: Optional[int] = None
        self._sensors_health: Optional[int] = None
        self._sys_time_s: Optional[float] = None
        self._distance_m: Optional[float] = None
        self._range_min_m = 0.0
        self._range_max_m = 0.0
        self._range_time_s: Optional[float] = None
        self._ground_xyz: Optional[tuple[float, float, float]] = None
        self._launch_xyz: Optional[tuple[float, float, float]] = None
        self._launch_yaw_rad: Optional[float] = None
        self._panel_map: Optional[PanelMap] = None
        self._localization_results: dict[int, dict] = {}
        self._perception: Optional[PerceptionResult] = None
        self._perception_time_s: Optional[float] = None
        self._ai_healthy = False
        self._distance_command: Optional[TwistStamped] = None
        self._distance_command_time_s: Optional[float] = None
        self._visual_command: Optional[TwistStamped] = None
        self._visual_command_time_s: Optional[float] = None
        self._visual_aligned = False
        self._visual_valid = False
        self._panel_visible = False
        self._distance_enabled = False
        self._distance_reached = False
        self._descend_target_latched = False
        self._takeoff_enabled = False
        self._takeoff_reached = False
        self._vertical_mode = 'DISABLED'
        self._altitude_guard_triggered = False
        self._spray_output_enabled: Optional[bool] = None
        self._spray_backend: Optional[str] = None
        self._spray_status_time_s: Optional[float] = None
        self._spray_cycle = SprayCycleTracker()
        self._post_spray_barrier: Optional[PerceptionBarrier] = None
        self._legacy_mission_state = 'IDLE'
        self._panel_mission_state = 'IDLE'
        self._state_started_s = now_s
        self._stage = 'IDLE'
        self._stage_started_s = self._state_started_s
        self._last_tick_s = now_s
        self._last_position_setpoint_s: Optional[float] = None
        self._last_velocity_setpoint_s: Optional[float] = None
        self._control_kind = 'none'
        self._command_xyz: Optional[tuple[float, float, float]] = None
        self._setpoint_speed_mps = 0.0
        self._last_setpoint_s = self._state_started_s
        self._survey_started_s: Optional[float] = None
        self._localization_scan_waypoints: list[tuple[float, float]] = []
        self._localization_scan_index = 0
        self._last_logged_scan_index = -1
        self._last_survey_progress_log_s = -float('inf')
        self._pending_action: Optional[str] = None
        self._last_action_s: dict[str, float] = {}
        self._spray_session_enabled = False
        self._spray_stop_requested = False
        self._abort_land_requested = False

    # -- telemetry callbacks -------------------------------------------------

    def _state_cb(self, message: State) -> None:
        previous_mode = self._mode
        self._connected = bool(message.connected)
        self._armed = bool(message.armed)
        self._mode = str(message.mode)
        if previous_mode and self._mode != previous_mode:
            now_s = time.monotonic()
            position_age = (
                None
                if self._last_position_setpoint_s is None
                else now_s - self._last_position_setpoint_s
            )
            velocity_age = (
                None
                if self._last_velocity_setpoint_s is None
                else now_s - self._last_velocity_setpoint_s
            )
            self.get_logger().warning(
                'PX4 mode transition: '
                f'{previous_mode} -> {self._mode}; '
                f'fsm={self._fsm.state.name}; stage={self._stage}; '
                f'control={self._control_kind}; '
                f'position_setpoint_age={position_age}; '
                f'velocity_setpoint_age={velocity_age}'
            )

    def _extended_cb(self, message: ExtendedState) -> None:
        self._landed_state = int(message.landed_state)
        self._extended_time_s = time.monotonic()
        self._capture_ground()

    def _estimator_cb(self, message: EstimatorStatus) -> None:
        self._estimator_attitude_valid = bool(message.attitude_status_flag)
        self._estimator_horizontal_velocity_valid = bool(
            message.velocity_horiz_status_flag
        )
        self._estimator_horizontal_relative_position_valid = bool(
            message.pos_horiz_rel_status_flag
        )
        self._estimator_horizontal_absolute_position_valid = bool(
            message.pos_horiz_abs_status_flag
        )
        self._estimator_constant_position_mode = bool(
            message.const_pos_mode_status_flag
        )
        self._estimator_time_s = time.monotonic()

    def _pose_cb(self, message: PoseStamped) -> None:
        position = message.pose.position
        values = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in values):
            return
        orientation = message.pose.orientation
        quaternion = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not all(math.isfinite(value) for value in quaternion) or norm <= 1e-6:
            return
        x, y, z, w = (value / norm for value in quaternion)
        self._yaw_rad = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        now_s = time.monotonic()
        self._pose_xyz = values
        self._pose_time_s = now_s
        if not self._armed:
            self._launch_yaw_stability.update(self._yaw_rad, now_s)
        self._capture_ground()

    def _velocity_cb(self, message: TwistStamped) -> None:
        linear = message.twist.linear
        values = (float(linear.x), float(linear.y), float(linear.z))
        if all(math.isfinite(value) for value in values):
            now_s = time.monotonic()
            self._velocity_xyz = values
            self._velocity_time_s = now_s
            self._survey_home_speed_filter.update(
                math.hypot(values[0], values[1]),
                now_s,
            )

    def _battery_cb(self, message: BatteryState) -> None:
        reported = float(message.percentage)
        reported = reported if math.isfinite(reported) else None
        voltage = float(message.voltage)
        current = float(message.current)
        self._battery_reported_remaining = reported
        self._battery_voltage_v = voltage if math.isfinite(voltage) else None
        self._battery_current_a = current if math.isfinite(current) else None
        compensated = sag_compensated_remaining(
            voltage=self._battery_voltage_v,
            current=self._battery_current_a,
            cell_count=self._battery_cell_count,
            internal_resistance_ohm=self._battery_internal_resistance_ohm,
            empty_volts_per_cell=self._battery_empty_volts_per_cell,
            full_volts_per_cell=self._battery_full_volts_per_cell,
        )
        remaining = compensated if compensated is not None else reported
        self._battery_remaining = remaining
        now_s = time.monotonic()
        self._battery_time_s = now_s
        if remaining is None or remaining >= self._minimum_battery:
            self._battery_below_since_s = None
        elif self._battery_below_since_s is None:
            self._battery_below_since_s = now_s

    def _sys_cb(self, message: SysStatus) -> None:
        self._sensors_enabled = int(message.sensors_enabled)
        self._sensors_health = int(message.sensors_health)
        self._sys_time_s = time.monotonic()

    def _range_cb(self, message: Range) -> None:
        distance_m = float(message.range)
        if not math.isfinite(distance_m):
            return
        self._distance_m = distance_m
        self._range_min_m = float(message.min_range)
        self._range_max_m = float(message.max_range)
        self._range_time_s = time.monotonic()

    def _panel_map_cb(self, message: PanelMap) -> None:
        self._panel_map = message

    def _perception_cb(self, message: PerceptionResult) -> None:
        self._perception = message
        self._perception_time_s = time.monotonic()

    def _ai_health_cb(self, message: Bool) -> None:
        self._ai_healthy = bool(message.data)

    def _distance_command_cb(self, message: TwistStamped) -> None:
        self._distance_command = message
        self._distance_command_time_s = time.monotonic()

    def _visual_command_cb(self, message: TwistStamped) -> None:
        self._visual_command = message
        self._visual_command_time_s = time.monotonic()

    def _visual_aligned_cb(self, message: Bool) -> None:
        self._visual_aligned = bool(message.data)

    def _visual_valid_cb(self, message: Bool) -> None:
        self._visual_valid = bool(message.data)

    def _panel_visible_cb(self, message: Bool) -> None:
        self._panel_visible = bool(message.data)

    def _distance_enabled_cb(self, message: Bool) -> None:
        self._distance_enabled = bool(message.data)

    def _distance_reached_cb(self, message: Bool) -> None:
        self._distance_reached = bool(message.data)

    def _takeoff_enabled_cb(self, message: Bool) -> None:
        self._takeoff_enabled = bool(message.data)

    def _takeoff_reached_cb(self, message: Bool) -> None:
        self._takeoff_reached = bool(message.data)

    def _vertical_mode_cb(self, message: String) -> None:
        self._vertical_mode = str(message.data)

    def _altitude_guard_cb(self, message: Bool) -> None:
        self._altitude_guard_triggered = bool(message.data)

    def _spray_status_cb(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            self._spray_output_enabled = bool(status['output_enabled'])
            self._spray_session_enabled = bool(status['session_enabled'])
            self._spray_backend = str(status['backend'])
            self._spray_status_time_s = time.monotonic()
        except (json.JSONDecodeError, KeyError, TypeError):
            self._spray_output_enabled = None
            self._spray_backend = None

    def _legacy_mission_cb(self, message: String) -> None:
        self._legacy_mission_state = str(message.data)

    def _panel_mission_cb(self, message: String) -> None:
        self._panel_mission_state = str(message.data)

    def _capture_ground(self) -> None:
        if (
            not self._armed
            and self._landed_state == self.ON_GROUND
            and self._pose_xyz is not None
        ):
            self._ground_xyz = self._pose_xyz

    # -- mission lifecycle ---------------------------------------------------

    def _start_callback(self, _request, response):
        if not self._configuration_approved:
            response.success = False
            response.message = 'configuration_approved=false'
            return response
        if not self._calibration_approved:
            response.success = False
            response.message = 'calibration_approved=false'
            return response
        if self._fsm.active:
            response.success = False
            response.message = f'mission already active in {self._fsm.state.name}'
            return response
        self._fsm.start()
        self._reset_mission_bookkeeping()
        self._publish_result('ACTIVE')
        self._publish_state()
        response.success = True
        response.message = 'autonomous cleaning mission requested'
        return response

    def _abort_callback(self, _request, response):
        if not self._fsm.active:
            response.success = False
            response.message = 'no active mission'
            return response
        self._fail('operator abort requested', request_land=True)
        response.success = True
        response.message = 'abort latched; landing requested'
        return response

    def _reset_mission_bookkeeping(self) -> None:
        self._survey_diagnostics = {}
        self._battery_below_since_s = None
        self._descend_target_latched = False
        now_s = time.monotonic()
        self._state_started_s = now_s
        self._stage_started_s = now_s
        self._last_tick_s = now_s
        self._last_position_setpoint_s = None
        self._last_velocity_setpoint_s = None
        self._stage = 'PRECHECK'
        self._control_kind = 'none'
        self._launch_xyz = None
        self._launch_yaw_rad = None
        self._panel_map = None
        self._localization_results.clear()
        self._command_xyz = None
        self._setpoint_speed_mps = 0.0
        self._survey_started_s = None
        self._localization_scan_waypoints = []
        self._localization_scan_index = 0
        self._last_logged_scan_index = -1
        self._last_survey_progress_log_s = -float('inf')
        self._pending_action = None
        self._last_action_s.clear()
        self._spray_session_enabled = False
        self._spray_cycle.reset()
        self._post_spray_barrier = None
        self._spray_stop_requested = False
        self._abort_land_requested = False
        self._arrival.reset()
        self._perception_window.reset()
        self._alignment_window.reset()
        self._survey_home_window.reset()
        self._survey_home_speed_filter.reset()
        self._launch_yaw_stability.reset()
        self._publish_survey(False)
        self._publish_ai_mode('idle')
        self._panel_id_publisher.publish(Int32(data=-1))

    def _tick(self) -> None:
        now_s = time.monotonic()
        state = self._fsm.state
        tick_gap_s = now_s - self._last_tick_s
        self._last_tick_s = now_s
        if (
            self._fsm.active
            and tick_gap_s > 0.25
            and state != CleaningMissionState.ABORT
        ):
            self.get_logger().warning(
                'Mission control tick delay: '
                f'{tick_gap_s:.3f} s; state={state.name}; '
                f'stage={self._stage}; control={self._control_kind}'
            )
        if state in {
            CleaningMissionState.IDLE,
            CleaningMissionState.COMPLETE,
        }:
            return
        if self._launch_yaw_rad is not None:
            self._yaw_target_publisher.publish(
                Float32(data=self._launch_yaw_rad)
            )
        if state == CleaningMissionState.ABORT:
            self._tick_abort()
            return
        if (
            self._armed
            and state not in {
                CleaningMissionState.PRECHECK,
                CleaningMissionState.ARMING,
                CleaningMissionState.LAND,
            }
        ):
            battery_failure = inflight_battery_failure(
                now_s=now_s,
                timeout_s=self._status_timeout_s,
                battery_remaining=self._battery_remaining,
                battery_time_s=self._battery_time_s,
                minimum_battery_remaining=self._minimum_battery,
                below_since_s=self._battery_below_since_s,
                sustained_below_s=self._battery_sustained_below_s,
            )
            if battery_failure is not None:
                self._fail(
                    f'{battery_failure} '
                    f'({self._battery_telemetry_note()})',
                    True,
                )
                return
        if self._altitude_guard_triggered:
            self._fail('independent altitude guard triggered', True)
            return
        if state not in {
            CleaningMissionState.PRECHECK,
            CleaningMissionState.LAND,
        }:
            allow_constant_position = early_takeoff_constant_position_allowed(
                landed_on_ground=self._landed_state == self.ON_GROUND,
                offboard_takeoff_state=(
                    state == CleaningMissionState.TAKEOFF
                    and self._stage in {'ENTER_OFFBOARD', 'HOLD'}
                ),
                state_elapsed_s=max(0.0, now_s - self._stage_started_s),
                airborne_grace_s=self._takeoff_const_pos_grace_s,
            )
            estimator_failures = self._horizontal_estimator_failures(
                now_s,
                allow_constant_position_mode=allow_constant_position,
            )
            if estimator_failures:
                self._fail(
                    '; '.join(estimator_failures)
                    + f' ({self._estimator_flag_note()})',
                    self._armed,
                )
                return
        if state not in {
            CleaningMissionState.PRECHECK,
            CleaningMissionState.ARMING,
            CleaningMissionState.LAND,
        }:
            if not self._telemetry_fresh(now_s):
                self._fail('local pose or velocity telemetry timeout', True)
                return
            if not self._range_healthy(now_s):
                self._fail('distance sensor timeout or invalid range', True)
                return
        elapsed_s = now_s - self._state_started_s
        if state == CleaningMissionState.SURVEY:
            elapsed_s = survey_timeout_elapsed_s(
                now_s,
                self._state_started_s,
                self._survey_started_s,
            )
        if elapsed_s > self._state_timeout_for(state):
            self._fail(f'{state.name} timeout', True)
            return
        handlers = {
            CleaningMissionState.PRECHECK: self._tick_precheck,
            CleaningMissionState.ARMING: self._tick_arming,
            CleaningMissionState.TAKEOFF: self._tick_takeoff,
            CleaningMissionState.SURVEY: self._tick_survey,
            CleaningMissionState.PLAN_ROUTE: self._tick_plan_route,
            CleaningMissionState.DESCEND: self._tick_descend,
            CleaningMissionState.TRANSIT: self._tick_position_state,
            CleaningMissionState.SLOW_APPROACH: self._tick_position_state,
            CleaningMissionState.REACQUIRE: self._tick_reacquire,
            CleaningMissionState.ASSESS: self._tick_assess,
            CleaningMissionState.PRECISION_ALIGN: self._tick_align,
            CleaningMissionState.SPRAY: self._tick_spray,
            CleaningMissionState.POST_SPRAY_ALIGN: self._tick_post_spray_align,
            CleaningMissionState.VERIFY: self._tick_verify,
            CleaningMissionState.RETURN_HOME: self._tick_return_home,
            CleaningMissionState.LAND: self._tick_land,
        }
        handler = handlers.get(state)
        if handler is not None:
            handler(now_s)

    def _state_timeout_for(self, state: CleaningMissionState) -> float:
        if state == CleaningMissionState.TAKEOFF:
            return self._takeoff_timeout_s
        if state == CleaningMissionState.SURVEY:
            return self._survey_timeout_s
        if state == CleaningMissionState.REACQUIRE:
            return self._reacquire_timeout_s
        if state in {
            CleaningMissionState.POST_SPRAY_ALIGN,
            CleaningMissionState.VERIFY,
        }:
            return self._verification_timeout_s
        return self._state_timeout_s

    def _tick_precheck(self, now_s: float) -> None:
        failures = self._precheck_failures(now_s)
        if failures:
            self._fail('preflight failed: ' + '; '.join(failures), False)
            return
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        stable_yaw = self._launch_yaw_stability.stable_yaw_rad
        if stable_yaw is None:
            return
        self._launch_xyz = self._ground_xyz
        self._launch_yaw_rad = stable_yaw
        previous = self._fsm.state
        self._fsm.precheck_complete()
        self._on_transition(previous)

    def _precheck_failures(self, now_s: float) -> list[str]:
        failures = []
        if not self._configuration_approved:
            failures.append('configuration approval is locked')
        if not self._calibration_approved:
            failures.append('calibration approval is locked')
        failures.extend(
            status_failures(
                now_s=now_s,
                timeout_s=self._status_timeout_s,
                battery_remaining=self._battery_remaining,
                battery_time_s=self._battery_time_s,
                minimum_battery_remaining=self._minimum_battery,
                landed_state=self._landed_state,
                extended_state_time_s=self._extended_time_s,
                require_on_ground=True,
                sensors_enabled=self._sensors_enabled,
                sensors_health=self._sensors_health,
                sys_status_time_s=self._sys_time_s,
                require_enabled_sensors_healthy=self._require_health,
                ignored_unhealthy_sensor_mask=self._ignored_health,
            )
        )
        failures.extend(
            self._horizontal_estimator_failures(
                now_s,
                allow_constant_position_mode=(
                    self._landed_state == self.ON_GROUND
                ),
            )
        )
        if not self._connected:
            failures.append('MAVROS disconnected')
        if self._armed:
            failures.append('vehicle already armed')
        if not self._telemetry_fresh(now_s):
            failures.append('local pose/velocity telemetry stale')
        if not self._range_healthy(now_s):
            failures.append('distance sensor unavailable')
        if not self._ai_healthy:
            failures.append(
                'Pi CPU panel heartbeat is not healthy'
                if self._localization_test_mode
                else 'laptop AI heartbeat is not healthy'
            )
        if self._ground_xyz is None:
            failures.append('launch Local XYZ reference unavailable')
        if self._vertical_mode != 'DISABLED':
            failures.append(f'vertical controller busy ({self._vertical_mode})')
        inactive = {'', 'IDLE', 'COMPLETE', 'ABORT'}
        if self._legacy_mission_state not in inactive:
            failures.append(f'legacy mission active ({self._legacy_mission_state})')
        if self._panel_mission_state not in inactive:
            failures.append(f'fixed panel mission active ({self._panel_mission_state})')
        velocity_publishers = self.count_publishers(
            '/mavros/setpoint_velocity/cmd_vel'
        )
        position_publishers = self.count_publishers(
            '/mavros/setpoint_position/local'
        )
        if velocity_publishers != 1:
            failures.append(
                'expected this mission to be the only MAVROS velocity '
                f'publisher ({velocity_publishers} found)'
            )
        if position_publishers != 1:
            failures.append(
                'expected this mission to be the only MAVROS position '
                f'publisher ({position_publishers} found)'
            )
        if self._require_live_spray:
            if not self._spray_output_enabled:
                failures.append('live spray output is unavailable or blocked')
            if self._spray_backend != 'pixhawk':
                failures.append(
                    'live spray requires the Pixhawk Camera Trigger backend'
                )
        if (
            self._spray_status_time_s is None
            or now_s - self._spray_status_time_s > self._status_timeout_s
        ):
            failures.append('spray status is unavailable or stale')
        return failures

    def _tick_arming(self, _now_s: float) -> None:
        if self._armed:
            previous = self._fsm.state
            self._fsm.armed()
            self._on_transition(previous)
            return
        request = CommandBool.Request()
        request.value = True
        self._request('arm', self._arm_client, request, lambda result: result.success)

    def _tick_takeoff(self, now_s: float) -> None:
        if self._stage == 'ENABLE':
            self._request_bool(
                'takeoff_enable', self._takeoff_client, True
            )
            if self._takeoff_enabled:
                self._set_stage('PRESTREAM')
            return
        self._publish_combined_velocity(now_s, include_visual=False)
        if self._stage == 'PRESTREAM':
            if now_s - self._stage_started_s >= self._prestream_s:
                self._set_stage('ENTER_OFFBOARD')
            return
        if self._stage == 'ENTER_OFFBOARD':
            if self._mode == 'OFFBOARD':
                self._control_kind = 'velocity'
                self._set_stage('HOLD')
            else:
                self._request_mode('OFFBOARD')
            return
        if self._stage == 'HOLD':
            if self._mode != 'OFFBOARD':
                self._fail(
                    f'PX4/QGC mode override during takeoff: {self._mode}',
                    False,
                )
                return
            if self._takeoff_reached:
                # Latch completion before disabling the controller. The
                # controller publishes reached=False as soon as it disables,
                # so completion cannot remain gated by the live reached flag.
                self._set_stage('EXIT_OFFBOARD')
            return

        if self._stage == 'EXIT_OFFBOARD':
            self._request_bool(
                'takeoff_disable', self._takeoff_client, False
            )
            self._request_mode(self._loiter_mode)
            if not self._takeoff_enabled and self._mode == self._loiter_mode:
                self._control_kind = 'none'
                previous = self._fsm.state
                self._fsm.takeoff_complete()
                self._on_transition(previous)
            return

        self._fail(f'unknown takeoff stage: {self._stage}', True)

    def _tick_survey(self, now_s: float) -> None:
        if not self._ai_healthy:
            self._fail(
                'Pi CPU panel heartbeat lost during survey'
                if self._localization_test_mode
                else 'laptop AI heartbeat lost during survey',
                True,
            )
            return
        if self._tick_position_control_entry(now_s):
            return
        if self._mode != 'OFFBOARD':
            self._fail(
                f'PX4/QGC mode override during survey: {self._mode}',
                False,
            )
            return
        if (
            self._launch_xyz is None
            or self._launch_yaw_rad is None
            or self._pose_xyz is None
            or self._yaw_rad is None
            or self._distance_m is None
        ):
            self._fail('survey launch reference unavailable', True)
            return
        target_xyz = self._panel_target_xyz(
            self._launch_xyz[0],
            self._launch_xyz[1],
            target_distance_m=self._survey_distance_m,
        )
        if self._localization_test_mode and self._survey_started_s is not None:
            if not self._localization_scan_waypoints:
                launch_x, launch_y = self._launch_xyz[:2]
                self._localization_scan_waypoints = list(
                    rectangular_scan_waypoints(
                        launch_x,
                        launch_y,
                        self._localization_scan_width_m,
                        self._localization_scan_depth_m,
                    )
                )
            scan_x, scan_y = self._localization_scan_waypoints[
                self._localization_scan_index
            ]
            target_xyz = self._panel_target_xyz(
                scan_x, scan_y, target_distance_m=self._survey_distance_m
            )
        self._advance_and_publish_position(target_xyz, now_s)
        if self._survey_started_s is None:
            filtered_speed = self._survey_home_speed_filter.value(now_s)
            position_error_m = math.hypot(
                self._pose_xyz[0] - self._launch_xyz[0],
                self._pose_xyz[1] - self._launch_xyz[1],
            )
            yaw_error_rad = wrapped_yaw_error(
                self._launch_yaw_rad, self._yaw_rad
            )
            stable = bool(
                filtered_speed is not None
                and position_error_m <= self._survey_home_tolerance_m
                and filtered_speed <= self._survey_home_max_speed_mps
                and abs(self._distance_m - self._survey_distance_m)
                <= self._lidar_z_tolerance_m
                and abs(yaw_error_rad) <= self._survey_home_yaw_tolerance_rad
            )
            if now_s - self._last_survey_progress_log_s >= 2.0:
                self.get_logger().info(
                    'Survey launch hold: '
                    f'position_error={position_error_m:.3f} m, '
                    f'speed={filtered_speed}, '
                    f'lidar_error={self._distance_m - self._survey_distance_m:.3f} m, '
                    f'yaw_error={math.degrees(yaw_error_rad):.2f} deg, '
                    f'stable={stable}'
                )
                self._last_survey_progress_log_s = now_s
            if not self._survey_home_window.update(stable, now_s):
                return
            self._survey_started_s = now_s
            self._panel_map = None
            if self._localization_test_mode:
                if (
                    self._localization_scan_yaw_aligned
                    and self._launch_yaw_rad is None
                ):
                    # A silent fall back to the world-aligned rectangle would
                    # scan the wrong ground without saying so.
                    self._fail(
                        'yaw-aligned scan requested but launch yaw is unknown',
                        True,
                    )
                    return
                self._localization_scan_waypoints = list(
                    rectangular_scan_waypoints(
                        self._launch_xyz[0],
                        self._launch_xyz[1],
                        self._localization_scan_width_m,
                        self._localization_scan_depth_m,
                        yaw_rad=(
                            self._launch_yaw_rad
                            if self._localization_scan_yaw_aligned
                            else None
                        ),
                        forward_offset_m=(
                            self._localization_scan_forward_offset_m
                        ),
                        lateral_offset_m=(
                            self._localization_scan_lateral_offset_m
                        ),
                    )
                )
                self._localization_scan_index = 0
                self._last_logged_scan_index = -1
                self.get_logger().info(
                    'Localization scan started with full timeout budget; '
                    f'launch=({self._launch_xyz[0]:.3f}, '
                    f'{self._launch_xyz[1]:.3f}); '
                    f'waypoints={self._localization_scan_waypoints}'
                )
            self._publish_ai_mode('survey')
            self._publish_survey(True)
            return
        if self._localization_test_mode:
            scan_x, scan_y = self._localization_scan_waypoints[
                self._localization_scan_index
            ]
            if self._localization_scan_index != self._last_logged_scan_index:
                self.get_logger().info(
                    'Localization scan waypoint '
                    f'{self._localization_scan_index + 1}/'
                    f'{len(self._localization_scan_waypoints)}; '
                    f'target=({scan_x:.3f}, {scan_y:.3f})'
                )
                self._last_logged_scan_index = self._localization_scan_index
            filtered_speed = self._survey_home_speed_filter.value(now_s)
            scan_error_m = math.hypot(
                self._pose_xyz[0] - scan_x,
                self._pose_xyz[1] - scan_y,
            )
            scan_yaw_error_deg = math.degrees(
                wrapped_yaw_error(self._launch_yaw_rad, self._yaw_rad)
            )
            if now_s - self._last_survey_progress_log_s >= 2.0:
                self.get_logger().info(
                    'Localization scan progress: '
                    f'waypoint={self._localization_scan_index + 1}/'
                    f'{len(self._localization_scan_waypoints)}, '
                    f'pose=({self._pose_xyz[0]:.3f}, '
                    f'{self._pose_xyz[1]:.3f}), '
                    f'command=({self._command_xyz[0]:.3f}, '
                    f'{self._command_xyz[1]:.3f}), '
                    f'target=({scan_x:.3f}, {scan_y:.3f}), '
                    f'error={scan_error_m:.3f} m, speed={filtered_speed}, '
                    f'yaw={math.degrees(self._yaw_rad):.2f} deg, '
                    f'yaw_error={scan_yaw_error_deg:.2f} deg, '
                    f'lidar={self._distance_m:.3f} m'
                )
                self._last_survey_progress_log_s = now_s
            if not self._arrival.update(
                position_error_m=scan_error_m,
                speed_mps=(
                    filtered_speed
                    if filtered_speed is not None
                    else math.inf
                ),
                now_s=now_s,
                telemetry_valid=filtered_speed is not None,
            ):
                return
            if self._localization_scan_index < len(
                self._localization_scan_waypoints
            ) - 1:
                self._localization_scan_index += 1
                self._arrival.reset()
                return
            self._publish_survey(False)
        elif now_s - self._survey_started_s < self._survey_duration_s:
            return
        else:
            self._publish_survey(False)
        if self._panel_map is None or not self._panel_map.finalized:
            return
        targets = self._panel_targets_from_message(self._panel_map)
        # Record the survey outcome before any abort. The abort path reports
        # the cleaning list, which is still empty here, so every 2026-08-23
        # abort logged "panels_total: 0" even when the survey had found
        # panels and failed only on the configured limit.
        self._survey_diagnostics = {
            'survey_panels_found': len(targets),
            'survey_panel_limit': self._maximum_survey_panels,
            'survey_panel_observations': [
                target.observation_count for target in targets
            ],
            'survey_panel_confidence': [
                round(target.confidence, 3) for target in targets
            ],
        }
        if not targets:
            self._fail('3 m survey produced no stable panel targets', True)
            return
        if len(targets) > self._maximum_survey_panels:
            self._fail(
                'survey panel count exceeds configured limit '
                f'({len(targets)} > {self._maximum_survey_panels}, '
                'observations '
                f'{[target.observation_count for target in targets]})',
                True,
            )
            return
        if (
            self._localization_test_mode
            and self._required_survey_panels > 0
            and len(targets) != self._required_survey_panels
        ):
            self._fail(
                'localization test requires exactly '
                f'{self._required_survey_panels} stable panel targets '
                f'(found {len(targets)})',
                True,
            )
            return
        previous = self._fsm.state
        self._fsm.survey_complete(targets)
        self._on_transition(previous)

    def _tick_plan_route(self, _now_s: float) -> None:
        if self._launch_xyz is None or self._pose_xyz is None:
            self._fail('route planning lacks launch/current pose', True)
            return
        try:
            plan = plan_panel_route(
                (self._pose_xyz[0], self._pose_xyz[1]),
                [progress.target for progress in self._fsm.panels],
                (self._launch_xyz[0], self._launch_xyz[1]),
            )
            previous = self._fsm.state
            self._fsm.route_planned(plan.panel_ids)
            self._on_transition(previous)
            self.get_logger().info(
                f'Dynamic route {plan.panel_ids}, '
                f'travel={plan.total_distance_m:.2f} m'
            )
        except ValueError as exc:
            self._fail(f'route planning failed: {exc}', True)

    def _tick_descend(self, now_s: float) -> None:
        if self._tick_velocity_control_entry(now_s, require_visual=False):
            return
        # Latch the arrival instead of re-testing the live flag, so the
        # handover is retried every tick the way TAKEOFF's EXIT_OFFBOARD
        # stage already does.
        #
        # Two mechanisms combine to deadlock the old single-shot version.
        # _handover_velocity_to_loiter() issues three service requests back
        # to back, but _request() drops any call made while another is still
        # in flight, so only the first one - distance_disable - is ever sent;
        # takeoff_disable and the AUTO.LOITER mode request are discarded.
        # Disabling the distance controller then clears _distance_reached on
        # the next sample, so the gate closes and nothing is ever retried.
        # The mission waits on self._mode == self._loiter_mode for a mode
        # request that was never transmitted.
        #
        # Flight 7 on 2026-08-24 is exactly this: target reached at t+259.942,
        # lost at t+259.971, 'Distance control disabled' at t+259.974, no
        # LOITER transition ever logged, DESCEND timeout at t+284.18.
        #
        # The handover is checked BEFORE the mode guard below, and the guard
        # exempts the loiter mode once the arrival is latched. Both are
        # required, because past that latch DESCEND is the thing asking for
        # AUTO.LOITER - so the guard would otherwise fire on our own request
        # at the exact moment it succeeds.
        #
        # That is what flights 2 and 3 on 2026-08-24 did. PX4 left OFFBOARD
        # ~12 s into DESCEND ACTIVE and the mission called it an outside
        # override, but the ULog shows DO_SET_MODE(AUTO) arriving from
        # source_component 191 - our own MAVROS - with no failsafe flag
        # before it and the offboard streams still healthy at 10 Hz. The 12 s
        # is _request() dropping in-flight duplicates until the mode request
        # finally goes out. Fixing the deadlock in c65e1bb made the request
        # reach PX4; it did not make the completion reachable.
        if self._distance_reached:
            self._descend_target_latched = True
        if self._descend_target_latched:
            self._handover_velocity_to_loiter()
            if not self._distance_enabled and self._mode == self._loiter_mode:
                self._control_kind = 'none'
                previous = self._fsm.state
                self._fsm.descent_complete()
                self._on_transition(previous)
                return
        if self._mode != 'OFFBOARD':
            if not (
                self._descend_target_latched
                and self._mode == self._loiter_mode
            ):
                self._fail(
                    f'PX4/QGC mode override during descent: {self._mode}',
                    False,
                )
            return
        self._publish_combined_velocity(now_s, include_visual=False)

    def _tick_position_state(self, now_s: float) -> None:
        if self._tick_position_control_entry(now_s):
            return
        if self._mode != 'OFFBOARD':
            self._fail(f'PX4/QGC mode override during transit: {self._mode}', False)
            return
        panel = self._fsm.current_panel
        if panel is None:
            self._fail('panel transit has no current panel', True)
            return
        if self._panel_visible and self._fsm.state == CleaningMissionState.TRANSIT:
            previous = self._fsm.state
            self._fsm.panel_visible()
            self._on_transition(previous, preserve_control=True)
        target_xyz = self._panel_target_xyz(panel.target.east_m, panel.target.north_m)
        self._advance_and_publish_position(target_xyz, now_s)
        arrived = self._arrival.update(
            position_error_m=math.hypot(
                self._pose_xyz[0] - panel.target.east_m,
                self._pose_xyz[1] - panel.target.north_m,
            ),
            speed_mps=self._horizontal_speed(),
            now_s=now_s,
        )
        if arrived:
            previous = self._fsm.state
            self._fsm.transit_arrived()
            self._on_transition(previous, preserve_control=True)

    def _tick_reacquire(self, now_s: float) -> None:
        if self._tick_position_control_entry(now_s):
            return
        if self._mode != 'OFFBOARD':
            self._fail(f'PX4/QGC mode override in reacquisition: {self._mode}', False)
            return
        panel = self._fsm.current_panel
        if panel is None:
            self._fail('reacquisition has no panel', True)
            return
        elapsed_s = now_s - self._state_started_s
        angle = 2.0 * math.pi * elapsed_s / self._reacquire_period_s
        radius = self._reacquire_radius_m * min(1.0, elapsed_s / 2.0)
        target_xyz = self._panel_target_xyz(
            panel.target.east_m + radius * math.cos(angle),
            panel.target.north_m + radius * math.sin(angle),
        )
        self._advance_and_publish_position(target_xyz, now_s)
        valid = self._fresh_clean_perception(now_s) and self._panel_visible
        if self._perception_window.update(valid, now_s):
            previous = self._fsm.state
            self._fsm.panel_reacquired()
            self._on_transition(previous, preserve_control=True)

    def _tick_assess(self, now_s: float) -> None:
        if self._mode != 'OFFBOARD' or self._control_kind != 'position':
            self._fail('position control lost during cleanliness assessment', False)
            return
        panel = self._fsm.current_panel
        if panel is None:
            self._fail('assessment has no panel', True)
            return
        self._publish_position_hold(self._panel_target_xyz(
            panel.target.east_m, panel.target.north_m
        ))
        if self._clean_packet_fresh(now_s) and not self._panel_visible:
            previous = self._fsm.state
            self._fsm.target_lost()
            self._on_transition(previous)
            return
        valid = self._fresh_clean_perception(now_s) and self._panel_visible
        if not valid:
            self._perception_window.reset()
            return
        if self._localization_test_mode:
            ready = self._alignment_conditions_ready()
        else:
            ready = (
                self._distance_and_yaw_ready()
                and self._horizontal_speed() <= self._arrival_max_speed_mps
            )
        if not ready:
            return
        if not self._perception_window.update(True, now_s):
            return
        if self._localization_test_mode:
            panel = self._fsm.current_panel
            if panel is None or self._pose_xyz is None:
                self._fail('localization result lacks panel or pose', True)
                return
            error_m = math.hypot(
                self._pose_xyz[0] - panel.target.east_m,
                self._pose_xyz[1] - panel.target.north_m,
            )
            self._localization_results[panel.target.panel_id] = {
                'located': bool(
                    error_m <= self._localization_position_tolerance_m
                ),
                'position_error_m': round(error_m, 3),
                'tolerance_m': self._localization_position_tolerance_m,
            }
            dirt_found = False
        else:
            dirt_found = bool(self._perception.dirt_found)
        previous = self._fsm.state
        self._fsm.cleanliness_result(dirt_found)
        self._on_transition(previous)

    def _tick_align(self, now_s: float) -> None:
        if self._tick_velocity_control_entry(now_s, require_visual=True):
            return
        if self._mode != 'OFFBOARD':
            self._fail(f'PX4/QGC mode override during alignment: {self._mode}', False)
            return
        if not self._fresh_clean_perception(now_s) or not self._panel_visible:
            previous = self._fsm.state
            self._fsm.target_lost()
            self._on_transition(previous)
            return
        self._publish_combined_velocity(now_s, include_visual=True)
        if self._alignment_window.update(
            self._alignment_conditions_ready(), now_s
        ):
            previous = self._fsm.state
            self._fsm.alignment_complete()
            self._on_transition(previous, preserve_control=True)

    def _tick_spray(self, now_s: float) -> None:
        self._publish_combined_velocity(now_s, include_visual=False)
        if (
            self._spray_status_time_s is None
            or now_s - self._spray_status_time_s > self._status_timeout_s
        ):
            self._fail('spray status communication timeout', True)
            return
        if self._spray_cycle.complete_if_elapsed(
            now_s,
            self._spray_duration_s,
        ):
            self._post_spray_barrier = self._current_perception_barrier()
            previous = self._fsm.state
            self._fsm.spray_complete()
            self._on_transition(previous, preserve_control=True)
            return
        if not self._spray_session_enabled:
            self._request_bool(
                'spray_enable', self._spray_enable_client, True
            )
            return
        if self._spray_cycle.trigger_requested:
            return
        self._request(
            'spray_trigger',
            self._spray_trigger_client,
            Trigger.Request(),
            lambda result: result.success,
            on_success=lambda _result: self._spray_cycle.accept_trigger(
                time.monotonic()
            ),
            on_dispatched=lambda: self._spray_cycle.latch_trigger(now_s),
            on_failure=lambda message: self._fail(
                f'spray trigger failed: {message}', True
            ),
        )

    def _tick_post_spray_align(self, now_s: float) -> None:
        fresh = self._fresh_post_spray_perception(now_s)
        if self._tick_velocity_control_entry(
            now_s,
            require_visual=fresh,
        ):
            return
        if self._mode != 'OFFBOARD':
            self._fail(
                f'PX4/QGC mode override during post-spray alignment: '
                f'{self._mode}',
                False,
            )
            return
        if not fresh:
            self._alignment_window.reset()
            return
        if not self._panel_visible:
            previous = self._fsm.state
            self._fsm.target_lost()
            self._on_transition(previous)
            return
        self._publish_combined_velocity(now_s, include_visual=True)
        if not self._alignment_window.update(
            self._alignment_conditions_ready(), now_s
        ):
            return
        previous = self._fsm.state
        self._fsm.post_spray_alignment_complete()
        self._on_transition(previous, preserve_control=True)

    def _tick_verify(self, now_s: float) -> None:
        fresh = self._fresh_post_spray_perception(now_s)
        self._publish_combined_velocity(now_s, include_visual=fresh)
        if not fresh:
            self._perception_window.reset()
            return
        if not self._panel_visible:
            previous = self._fsm.state
            self._fsm.target_lost()
            self._on_transition(previous)
            return
        if not self._alignment_conditions_ready():
            self._perception_window.reset()
            return
        if not self._perception_window.update(True, now_s):
            return
        dirt_found = bool(self._perception.dirt_found)
        previous = self._fsm.state
        self._fsm.cleanliness_result(dirt_found)
        self._on_transition(previous)

    def _tick_return_home(self, now_s: float) -> None:
        if self._tick_position_control_entry(now_s):
            return
        if self._launch_xyz is None:
            self._fail('home reference unavailable', True)
            return
        target_xyz = self._panel_target_xyz(
            self._launch_xyz[0], self._launch_xyz[1]
        )
        self._advance_and_publish_position(target_xyz, now_s)
        arrived = self._arrival.update(
            position_error_m=math.hypot(
                self._pose_xyz[0] - self._launch_xyz[0],
                self._pose_xyz[1] - self._launch_xyz[1],
            ),
            speed_mps=self._horizontal_speed(),
            now_s=now_s,
        )
        if arrived:
            self._request_mode(self._loiter_mode)
            if self._mode == self._loiter_mode:
                previous = self._fsm.state
                self._fsm.home_arrived()
                self._on_transition(previous)

    def _tick_land(self, _now_s: float) -> None:
        self._publish_survey(False)
        self._publish_ai_mode('idle')
        self._request_bool('spray_disable', self._spray_enable_client, False)
        if self._mode != self._land_mode and self._armed:
            self._request_mode(self._land_mode)
            return
        if not self._armed and self._landed_state == self.ON_GROUND:
            previous = self._fsm.state
            self._fsm.landed()
            self._on_transition(previous)
            self._publish_mission_summary('COMPLETE')

    def _tick_abort(self) -> None:
        self._publish_survey(False)
        self._publish_ai_mode('idle')
        self._request_spray_stop_once()
        self._request_bool('takeoff_disable', self._takeoff_client, False)
        self._request_bool('distance_disable', self._distance_client, False)
        self._request_bool('spray_disable', self._spray_enable_client, False)
        if self._armed and self._abort_land_requested and self._mode != self._land_mode:
            self._request_mode(self._land_mode)

    # -- control mode handovers ---------------------------------------------

    def _tick_velocity_control_entry(
        self,
        now_s: float,
        *,
        require_visual: bool,
    ) -> bool:
        if self._control_kind == 'velocity' and self._stage == 'ACTIVE':
            self._publish_combined_velocity(now_s, include_visual=require_visual)
            return False
        if self._stage == 'HANDOVER':
            if self._mode != self._loiter_mode:
                self._request_mode(self._loiter_mode)
                return True
            self._request_bool('distance_enable', self._distance_client, True)
            if self._distance_enabled:
                self._set_stage('PRESTREAM')
            return True
        self._publish_combined_velocity(now_s, include_visual=require_visual)
        if self._stage == 'PRESTREAM':
            if now_s - self._stage_started_s >= self._prestream_s:
                self._set_stage('ENTER_OFFBOARD')
            return True
        if self._stage == 'ENTER_OFFBOARD':
            if self._mode == 'OFFBOARD':
                self._control_kind = 'velocity'
                self._set_stage('ACTIVE')
                return False
            self._request_mode('OFFBOARD')
            return True
        return False

    def _tick_position_control_entry(self, now_s: float) -> bool:
        if self._control_kind == 'position' and self._stage == 'ACTIVE':
            return False
        if self._stage == 'HANDOVER':
            self._request_bool('distance_disable', self._distance_client, False)
            self._request_bool('takeoff_disable', self._takeoff_client, False)
            self._request_bool('spray_disable', self._spray_enable_client, False)
            if self._mode != self._loiter_mode:
                self._request_mode(self._loiter_mode)
                return True
            if self._distance_enabled or self._takeoff_enabled:
                return True
            if self._pose_xyz is None:
                return True
            self._command_xyz = self._pose_xyz
            self._setpoint_speed_mps = 0.0
            self._last_setpoint_s = now_s
            self._set_stage('PRESTREAM')
        if self._stage == 'PRESTREAM':
            self._publish_position_hold(self._command_xyz)
            if now_s - self._stage_started_s >= self._prestream_s:
                self._set_stage('ENTER_OFFBOARD')
            return True
        if self._stage == 'ENTER_OFFBOARD':
            self._publish_position_hold(self._command_xyz)
            if self._mode == 'OFFBOARD':
                self._control_kind = 'position'
                self._set_stage('ACTIVE')
                return False
            self._request_mode('OFFBOARD')
            return True
        return False

    def _handover_velocity_to_loiter(self) -> None:
        self._request_bool('distance_disable', self._distance_client, False)
        self._request_bool('takeoff_disable', self._takeoff_client, False)
        self._request_mode(self._loiter_mode)

    def _publish_combined_velocity(
        self,
        now_s: float,
        *,
        include_visual: bool,
    ) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        distance_fresh = (
            self._distance_command is not None
            and self._distance_command_time_s is not None
            and now_s - self._distance_command_time_s <= self._sensor_timeout_s
        )
        if distance_fresh:
            command.twist.linear.z = self._distance_command.twist.linear.z
            command.twist.angular.z = self._distance_command.twist.angular.z
        visual_fresh = (
            self._visual_command is not None
            and self._visual_command_time_s is not None
            and now_s - self._visual_command_time_s <= self._perception_timeout_s
        )
        if include_visual and visual_fresh and self._visual_valid:
            command.twist.linear.x = self._visual_command.twist.linear.x
            command.twist.linear.y = self._visual_command.twist.linear.y
        self._velocity_publisher.publish(command)
        self._last_velocity_setpoint_s = time.monotonic()

    def _panel_target_xyz(
        self,
        east_m: float,
        north_m: float,
        *,
        target_distance_m: Optional[float] = None,
    ):
        if self._pose_xyz is None or self._distance_m is None:
            raise RuntimeError('pose/range unavailable for target')
        if target_distance_m is None:
            target_distance_m = self._spray_distance_m
        local_z = lidar_referenced_local_z_target(
            local_z_m=self._pose_xyz[2],
            measured_distance_m=self._distance_m,
            target_distance_m=target_distance_m,
            gain=self._lidar_z_gain,
            maximum_offset_m=self._lidar_z_max_offset_m,
            tolerance_m=self._lidar_z_control_deadband_m,
        )
        return east_m, north_m, local_z

    def _advance_and_publish_position(
        self,
        target_xyz: tuple[float, float, float],
        now_s: float,
    ) -> None:
        if self._command_xyz is None:
            self._command_xyz = self._pose_xyz
        dt_s = max(0.001, min(0.2, now_s - self._last_setpoint_s))
        self._last_setpoint_s = now_s
        speed_limit = (
            self._visible_speed_mps
            if self._panel_visible
            else self._cruise_speed_mps
        )
        self._command_xyz, self._setpoint_speed_mps = (
            advance_slowed_position_setpoint(
                self._command_xyz,
                target_xyz,
                (self._pose_xyz[0], self._pose_xyz[1]),
                current_horizontal_speed_mps=self._setpoint_speed_mps,
                maximum_horizontal_speed_mps=speed_limit,
                maximum_horizontal_accel_mps2=self._horizontal_accel_mps2,
                horizontal_slow_zone_m=self._horizontal_slow_zone_m,
                minimum_approach_speed_mps=min(
                    self._minimum_approach_speed_mps,
                    speed_limit,
                ),
                target_snap_distance_m=self._target_snap_distance_m,
                maximum_vertical_speed_mps=self._maximum_vertical_speed_mps,
                dt_s=dt_s,
            )
        )
        self._publish_position_hold(self._command_xyz)

    def _publish_position_hold(self, xyz) -> None:
        if xyz is None or self._launch_yaw_rad is None:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = xyz[0]
        message.pose.position.y = xyz[1]
        message.pose.position.z = xyz[2]
        half_yaw = self._launch_yaw_rad / 2.0
        message.pose.orientation.z = math.sin(half_yaw)
        message.pose.orientation.w = math.cos(half_yaw)
        self._position_publisher.publish(message)
        self._last_position_setpoint_s = time.monotonic()

    # -- decisions and state bookkeeping ------------------------------------

    def _fresh_clean_perception(self, now_s: float) -> bool:
        panel = self._fsm.current_panel
        return bool(
            self._clean_packet_fresh(now_s)
            and self._perception.valid
            and (
                int(self._perception.active_panel_id) < 0
                or panel is None
                or int(self._perception.active_panel_id)
                == panel.target.panel_id
            )
        )

    def _fresh_post_spray_perception(self, now_s: float) -> bool:
        completed_s = self._spray_cycle.pulse_completed_s
        if (
            completed_s is None
            or self._perception is None
            or self._perception_time_s is None
            or self._perception_time_s <= completed_s
            or not self._fresh_clean_perception(now_s)
        ):
            return False
        return perception_is_newer(
            session_id=str(self._perception.session_id),
            sequence=int(self._perception.sequence),
            frame_id=int(self._perception.frame_id),
            barrier=self._post_spray_barrier,
        )

    def _current_perception_barrier(self) -> Optional[PerceptionBarrier]:
        if self._perception is None:
            return None
        return PerceptionBarrier(
            session_id=str(self._perception.session_id),
            sequence=int(self._perception.sequence),
            frame_id=int(self._perception.frame_id),
        )

    def _clean_packet_fresh(self, now_s: float) -> bool:
        return bool(
            self._ai_healthy
            and self._perception is not None
            and self._perception_time_s is not None
            and now_s - self._perception_time_s <= self._perception_timeout_s
            and str(self._perception.mode).lower() == 'clean'
        )

    def _distance_and_yaw_ready(self) -> bool:
        if self._distance_m is None or self._yaw_rad is None:
            return False
        if self._launch_yaw_rad is None:
            return False
        return (
            abs(self._distance_m - self._spray_distance_m)
            <= self._lidar_z_tolerance_m
            and abs(wrapped_yaw_error(self._launch_yaw_rad, self._yaw_rad))
            <= self._yaw_tolerance_rad
        )

    def _alignment_conditions_ready(self) -> bool:
        return bool(
            self._visual_valid
            and self._visual_aligned
            and self._distance_reached
            and self._distance_and_yaw_ready()
            and self._horizontal_speed() <= self._arrival_max_speed_mps
        )

    def _battery_telemetry_note(self) -> str:
        """Describe the pack so a sag abort is separable from a flat pack."""
        reported = self._battery_reported_remaining
        return (
            'px4='
            + ('n/a' if reported is None else f'{reported:.0%}')
            + ', V='
            + (
                'n/a'
                if self._battery_voltage_v is None
                else f'{self._battery_voltage_v:.2f}'
            )
            + ', I='
            + (
                'n/a'
                if self._battery_current_a is None
                else f'{self._battery_current_a:.1f}A'
            )
        )

    def _estimator_flag_note(self) -> str:
        """Name the estimator flags behind an abort, for post-flight triage."""
        def flag(value: Optional[bool]) -> str:
            return 'n/a' if value is None else ('ok' if value else 'BAD')

        return (
            'att=' + flag(self._estimator_attitude_valid)
            + ', vel=' + flag(self._estimator_horizontal_velocity_valid)
            + ', rel_pos='
            + flag(self._estimator_horizontal_relative_position_valid)
            + ', abs_pos='
            + flag(self._estimator_horizontal_absolute_position_valid)
            + ', const_pos='
            + flag(self._estimator_constant_position_mode)
        )

    def _panel_targets_from_message(self, message: PanelMap):
        return tuple(
            PanelTarget(
                panel_id=int(panel.panel_id),
                east_m=float(panel.east_m),
                north_m=float(panel.north_m),
                width_m=float(panel.width_m),
                height_m=float(panel.height_m),
                confidence=float(panel.confidence),
                observation_count=int(panel.observation_count),
            )
            for panel in message.panels
        )

    def _on_transition(
        self,
        previous: CleaningMissionState,
        preserve_control: bool = False,
    ) -> None:
        state = self._fsm.state
        if state == previous:
            return
        now_s = time.monotonic()
        self._state_started_s = now_s
        self._stage_started_s = now_s
        self._perception_window.reset()
        self._alignment_window.reset()
        self._arrival.reset()
        self._descend_target_latched = False
        if state == CleaningMissionState.SPRAY:
            self._spray_cycle.reset()
            self._post_spray_barrier = None
        if state == CleaningMissionState.SURVEY:
            self._survey_started_s = None
            self._survey_home_window.reset()
            self._survey_home_speed_filter.reset()
            self._localization_scan_waypoints = []
            self._localization_scan_index = 0
            self._last_logged_scan_index = -1
            self._last_survey_progress_log_s = -float('inf')
        if state == CleaningMissionState.TAKEOFF:
            self._stage = 'ENABLE'
            self._control_kind = 'none'
        elif state in {
            CleaningMissionState.DESCEND,
            CleaningMissionState.PRECISION_ALIGN,
            CleaningMissionState.POST_SPRAY_ALIGN,
        }:
            if preserve_control and self._control_kind == 'velocity':
                self._stage = 'ACTIVE'
            else:
                self._stage = 'HANDOVER'
                self._control_kind = 'none'
        elif state in self.POSITION_STATES:
            if preserve_control and self._control_kind == 'position':
                self._stage = 'ACTIVE'
            else:
                self._stage = 'HANDOVER'
                self._control_kind = 'none'
            self._command_xyz = self._pose_xyz
            self._setpoint_speed_mps = 0.0
        else:
            self._stage = 'ACTIVE'
        if state in {
            CleaningMissionState.TRANSIT,
            CleaningMissionState.SLOW_APPROACH,
            CleaningMissionState.REACQUIRE,
            CleaningMissionState.ASSESS,
            CleaningMissionState.PRECISION_ALIGN,
            CleaningMissionState.SPRAY,
            CleaningMissionState.POST_SPRAY_ALIGN,
            CleaningMissionState.VERIFY,
        }:
            panel = self._fsm.current_panel
            panel_id = panel.target.panel_id if panel else -1
            self._panel_id_publisher.publish(Int32(data=panel_id))
            self._publish_ai_mode('clean')
        if state in {
            CleaningMissionState.RETURN_HOME,
            CleaningMissionState.LAND,
            CleaningMissionState.COMPLETE,
            CleaningMissionState.ABORT,
        }:
            self._panel_id_publisher.publish(Int32(data=-1))
        self.get_logger().info(
            f'{previous.name} -> {state.name}: {self._fsm.reason}'
        )
        if (
            previous in {
                CleaningMissionState.ASSESS,
                CleaningMissionState.VERIFY,
            }
            and state in {
                CleaningMissionState.TRANSIT,
                CleaningMissionState.RETURN_HOME,
            }
            and self._fsm.current_index > 0
        ):
            self._publish_panel_result(
                self._fsm.panels[self._fsm.current_index - 1]
            )
        self._publish_state()

    def _set_stage(self, stage: str) -> None:
        if stage == self._stage:
            return
        self._stage = stage
        self._stage_started_s = time.monotonic()

    def _telemetry_fresh(self, now_s: float) -> bool:
        return (
            self._pose_xyz is not None
            and self._pose_time_s is not None
            and now_s - self._pose_time_s <= self._telemetry_timeout_s
            and self._velocity_xyz is not None
            and self._velocity_time_s is not None
            and now_s - self._velocity_time_s <= self._telemetry_timeout_s
        )

    def _horizontal_estimator_failures(
        self,
        now_s: float,
        *,
        allow_constant_position_mode: bool = False,
    ) -> list[str]:
        return horizontal_estimator_failures(
            now_s=now_s,
            timeout_s=self._status_timeout_s,
            estimator_time_s=self._estimator_time_s,
            attitude_valid=self._estimator_attitude_valid,
            horizontal_velocity_valid=(
                self._estimator_horizontal_velocity_valid
            ),
            horizontal_relative_position_valid=(
                self._estimator_horizontal_relative_position_valid
            ),
            horizontal_absolute_position_valid=(
                self._estimator_horizontal_absolute_position_valid
            ),
            constant_position_mode=self._estimator_constant_position_mode,
            allow_constant_position_mode=allow_constant_position_mode,
        )

    def _range_healthy(self, now_s: float) -> bool:
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

    def _horizontal_speed(self) -> float:
        if self._velocity_xyz is None:
            return math.inf
        return math.hypot(self._velocity_xyz[0], self._velocity_xyz[1])

    # -- service requests and fail-safe --------------------------------------

    def _request_mode(self, mode: str) -> None:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self._request(
            f'mode:{mode}',
            self._mode_client,
            request,
            lambda result: result.mode_sent,
        )

    def _request_bool(self, action: str, client, enabled: bool) -> None:
        request = SetBool.Request()
        request.data = enabled
        self._request(action, client, request, lambda result: result.success)

    def _request(
        self,
        action: str,
        client,
        request,
        accepted: Callable,
        on_success: Optional[Callable] = None,
        on_dispatched: Optional[Callable] = None,
        on_failure: Optional[Callable] = None,
    ) -> bool:
        now_s = time.monotonic()
        if self._pending_action is not None:
            return False
        if now_s - self._last_action_s.get(action, -math.inf) < self._action_retry_s:
            return False
        if not client.service_is_ready():
            self._last_action_s[action] = now_s
            return False
        self._pending_action = action
        self._last_action_s[action] = now_s
        try:
            future = client.call_async(request)
        except Exception as exc:  # ROS service dispatch error
            self._pending_action = None
            message = str(exc)
            self.get_logger().error(f'{action} service dispatch failed: {message}')
            if on_failure is not None:
                on_failure(message)
            return False
        if on_dispatched is not None:
            on_dispatched()

        def completed(result_future) -> None:
            self._pending_action = None
            try:
                result = result_future.result()
                success = bool(accepted(result))
            except Exception as exc:  # ROS service transport error
                message = str(exc)
                self.get_logger().error(f'{action} service failed: {message}')
                if on_failure is not None:
                    on_failure(message)
                return
            if not success:
                message = getattr(result, 'message', 'request rejected')
                self.get_logger().warning(f'{action} rejected: {message}')
                if on_failure is not None:
                    on_failure(message)
                return
            if on_success is not None:
                on_success(result)

        future.add_done_callback(completed)
        return True

    def _request_spray_stop_once(self) -> None:
        """Send PX4 trigger-disable independently of normal action retries."""
        if self._spray_stop_requested or not self._spray_stop_client.service_is_ready():
            return
        try:
            future = self._spray_stop_client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().error(f'spray stop dispatch failed: {exc}')
            return
        self._spray_stop_requested = True

        def completed(result_future) -> None:
            try:
                result = result_future.result()
                if not result.success:
                    self.get_logger().error(
                        f'spray stop rejected: {result.message}'
                    )
                    self._spray_stop_requested = False
            except Exception as exc:
                self.get_logger().error(f'spray stop service failed: {exc}')
                self._spray_stop_requested = False

        future.add_done_callback(completed)

    def _fail(self, reason: str, request_land: bool) -> None:
        if self._fsm.state == CleaningMissionState.ABORT:
            return
        self.get_logger().error(reason)
        self._request_spray_stop_once()
        self._fsm.abort(reason)
        self._abort_land_requested = bool(request_land)
        self._publish_mission_summary('ABORT', reason)
        self._publish_state()
        self._publish_survey(False)
        self._publish_ai_mode('idle')

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self._fsm.state.name))

    def _publish_result(self, result: str) -> None:
        self._result_publisher.publish(String(data=result))

    def _publish_readiness(self) -> None:
        now_s = time.monotonic()
        failures = self._precheck_failures(now_s)
        inactive = self._fsm.state in {
            CleaningMissionState.IDLE,
            CleaningMissionState.COMPLETE,
            CleaningMissionState.ABORT,
        }
        if not inactive:
            failures.insert(0, f'mission active ({self._fsm.state.name})')
        payload = {
            'ready': inactive and not failures,
            'failures': failures,
            'configuration_approved': self._configuration_approved,
            'calibration_approved': self._calibration_approved,
            'require_live_spray': self._require_live_spray,
        }
        self._readiness_publisher.publish(
            String(data=json.dumps(payload, separators=(',', ':')))
        )

    def _panel_result(self, progress) -> dict:
        result = {
            'panel_id': progress.target.panel_id,
            'spray_attempts': progress.spray_attempts,
            'clean': progress.clean,
            'cleaning_failed': progress.cleaning_failed,
            'failure_reason': progress.failure_reason,
        }
        if self._localization_test_mode:
            result['localization'] = self._localization_results.get(
                progress.target.panel_id,
                {'located': False, 'reason': 'not checked'},
            )
        return result

    def _publish_panel_result(self, progress) -> None:
        self._publish_result(json.dumps(
            {
                'status': 'PANEL_RESULT',
                'panel': self._panel_result(progress),
            },
            separators=(',', ':'),
        ))

    def _publish_mission_summary(self, status: str, reason: str = '') -> None:
        summary = {
            'status': status,
            'reason': reason,
            'panels_total': len(self._fsm.panels),
            'panels_clean': sum(panel.clean for panel in self._fsm.panels),
            'panels_cleaning_failed': sum(
                panel.cleaning_failed for panel in self._fsm.panels
            ),
            'panels': [
                self._panel_result(panel) for panel in self._fsm.panels
            ],
        }
        # panels_total counts the cleaning list, which an abort during survey
        # never fills. Carry the survey outcome separately so a run that
        # mapped panels is distinguishable from one that saw nothing.
        summary.update(self._survey_diagnostics)
        self._publish_result(json.dumps(summary, separators=(',', ':')))

    def _publish_survey(self, active: bool) -> None:
        self._survey_publisher.publish(Bool(data=active))

    def _publish_ai_mode(self, mode: str) -> None:
        self._ai_mode_publisher.publish(String(data=mode))


def main(args=None) -> None:
    """Run the full autonomous cleaning mission manager."""
    rclpy.init(args=args)
    node = AutonomousCleaningMissionNode()
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
