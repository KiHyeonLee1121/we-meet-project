"""Shared flight settings; no camera, perception, position target, or spray options."""
from dataclasses import dataclass, fields
import math
from pathlib import Path
import yaml


@dataclass(frozen=True)
class Config:
    advance_enabled: bool = True
    control_hz: float = 20.0
    target_height_m: float = 3.0
    commanded_target_distance_m: float = 5.0
    commanded_distance_tolerance_m: float = 0.15
    forward_speed_mps: float = 0.35
    horizontal_accel_mps2: float = 0.25
    vertical_kp: float = 0.6
    vertical_kd: float = 0.15
    vertical_speed_mps: float = 0.25
    vertical_accel_mps2: float = 0.5
    altitude_tolerance_m: float = 0.10
    altitude_rate_tolerance_mps: float = 0.05
    settle_s: float = 2.0
    zero_velocity_hold_s: float = 5.0
    yaw_kp: float = 1.0
    yaw_max_rate_rad_s: float = 0.35
    yaw_tolerance_rad: float = 0.0872664626
    yaw_stable_s: float = 1.0
    yaw_stable_deviation_rad: float = 0.034906585
    yaw_reset_min_rad: float = 0.026179939
    lidar_input_is_vertical_height: bool = False
    lidar_body_down_offset_m: float = 0.0
    maximum_tilt_rad: float = 0.35
    lidar_rate_window_s: float = 0.5
    sensor_timeout_s: float = 0.3
    state_timeout_s: float = 2.5
    landed_timeout_s: float = 2.0
    battery_timeout_s: float = 2.0
    maximum_tick_gap_s: float = 0.2
    prestream_s: float = 1.2
    service_timeout_s: float = 5.0
    ascend_timeout_s: float = 35.0
    settle_timeout_s: float = 15.0
    advance_timeout_s: float = 35.0
    brake_timeout_s: float = 5.0
    hold_timeout_s: float = 10.0
    mission_timeout_s: float = 110.0
    landing_timeout_s: float = 35.0
    minimum_battery_remaining: float = 0.15
    # Provisional acceptance/abort limits. Verify against the actual aircraft.
    fc_odometry_timeout_s: float = 0.3
    maximum_position_sigma_m: float = 0.5
    maximum_velocity_sigma_mps: float = 0.2
    gps_accuracy_required: bool = True
    gps_timeout_s: float = 2.0
    maximum_gps_sigma_m: float = 0.5
    horizontal_tracking_limit_m: float = 1.0
    horizontal_tracking_dwell_s: float = 0.5
    position_velocity_residual_limit_m: float = 0.5
    position_velocity_window_s: float = 1.0
    yaw_abort_error_rad: float = 0.261799388
    yaw_abort_rate_rad_s: float = 0.610865238
    yaw_abort_dwell_s: float = 0.5
    lidar_velocity_check_enabled: bool = True
    lidar_velocity_disagreement_mps: float = 0.25
    lidar_velocity_disagreement_s: float = 0.75
    hold_horizontal_speed_mps: float = 0.15
    hold_vertical_speed_mps: float = 0.10

    def __post_init__(self):
        for field in fields(Config):
            name, value = field.name, getattr(self, field.name)
            if name in ('advance_enabled', 'lidar_input_is_vertical_height', 'gps_accuracy_required', 'lidar_velocity_check_enabled'):
                if not isinstance(value, bool):
                    raise ValueError(f'{name} must be boolean')
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite numeric')
            if name == 'lidar_body_down_offset_m':
                continue
            if value <= 0:
                raise ValueError(f'{name} must be positive')
        if not 0 < self.minimum_battery_remaining < 1 or not 0 < self.maximum_tilt_rad < math.pi/4:
            raise ValueError('invalid battery/tilt limit')
        if self.maximum_tick_gap_s < 2/self.control_hz or self.prestream_s < 1.0:
            raise ValueError('insufficient control-gap allowance/prestream')
        if self.zero_velocity_hold_s >= self.hold_timeout_s:
            raise ValueError('hold timeout must exceed requested zero-velocity hold')
        if self.commanded_distance_tolerance_m >= self.commanded_target_distance_m:
            raise ValueError('command distance tolerance must be smaller than distance')
        if self.forward_speed_mps*self.maximum_tick_gap_s > self.commanded_distance_tolerance_m:
            raise ValueError('command distance tolerance must cover the configured maximum publish interval')
        if not self.yaw_tolerance_rad < self.yaw_abort_error_rad < math.pi:
            raise ValueError('yaw abort angle must exceed hold tolerance and be less than pi')
        if self.position_velocity_window_s <= self.fc_odometry_timeout_s:
            raise ValueError('position/velocity window must exceed odometry timeout')


def load_config(path):
    values = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(values, dict):
        raise ValueError('configuration must be a mapping')
    return Config(**values)
