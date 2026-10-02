from dataclasses import asdict, dataclass
import math
from pathlib import Path
import yaml


@dataclass(frozen=True)
class Config:
    vision_enabled: bool = True
    approach_enabled: bool = True
    lidar_input_is_vertical_height: bool = True
    lidar_body_down_offset_m: float = 0.0
    maximum_tilt_rad: float = 0.35
    control_hz: float = 20.0
    target_range_m: float = 3.0
    approach_distance_m: float = 5.0
    cruise_speed_mps: float = 0.35
    horizontal_accel_mps2: float = 0.25
    vertical_kp: float = 0.6
    vertical_kd: float = 0.15
    vertical_speed_mps: float = 0.25
    vertical_accel_mps2: float = 0.5
    altitude_tolerance_m: float = 0.12
    altitude_rate_tolerance_mps: float = 0.08
    settle_s: float = 2.0
    hover_s: float = 5.0
    yaw_kp: float = 1.0
    yaw_max_rate_rad_s: float = 0.35
    yaw_tolerance_rad: float = 0.0872664626
    yaw_stable_s: float = 1.0
    yaw_stable_deviation_rad: float = 0.034906585
    yaw_reset_min_rad: float = 0.026179939
    sensor_timeout_s: float = 0.3
    state_timeout_s: float = 2.5
    camera_timeout_s: float = 0.3
    target_loss_s: float = 0.6
    maximum_tick_gap_s: float = 0.2
    prestream_s: float = 1.2
    service_timeout_s: float = 5.0
    mission_timeout_s: float = 100.0
    landing_timeout_s: float = 35.0
    minimum_battery_remaining: float = 0.15
    visual_gain_mps: float = 0.35
    visual_max_speed_mps: float = 0.12
    center_half_width: float = 0.15
    center_half_height: float = 0.15
    # Image x/y -> body FLU forward/left. Confirm with the installed camera.
    image_to_body: tuple = ((0.0, -1.0), (-1.0, 0.0))
    min_panel_area_fraction: float = 0.004
    max_panel_area_fraction: float = 0.75
    min_panel_aspect: float = 1.0
    max_panel_aspect: float = 3.5
    min_panel_dark_fraction: float = 0.55
    panel_dark_threshold: int = 130
    acquisition_frames: int = 3
    association_radius_norm: float = 0.16
    max_frame_age_s: float = 0.3

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name in ('vision_enabled', 'approach_enabled', 'lidar_input_is_vertical_height',
                        'lidar_body_down_offset_m', 'image_to_body'):
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be numeric')
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not isinstance(self.vision_enabled, bool):
            raise ValueError('vision_enabled must be boolean')
        if not isinstance(self.approach_enabled, bool) or not isinstance(self.lidar_input_is_vertical_height, bool):
            raise ValueError('approach/lidar flags must be boolean')
        if not math.isfinite(self.lidar_body_down_offset_m):
            raise ValueError('lidar offset must be finite')
        if self.vision_enabled and not self.approach_enabled:
            raise ValueError('vision requires approach_enabled')
        matrix = self.image_to_body
        if len(matrix) != 2 or any(len(row) != 2 for row in matrix):
            raise ValueError('image_to_body must be a 2x2 matrix')
        if not all(math.isfinite(v) for row in matrix for v in row):
            raise ValueError('image_to_body must be finite')
        if any(abs(sum(v*v for v in row) - 1) > 1e-6 for row in matrix):
            raise ValueError('image_to_body rows must be unit length')
        if abs(sum(matrix[0][i]*matrix[1][i] for i in range(2))) > 1e-6:
            raise ValueError('image_to_body rows must be orthogonal')
        if not 0 < self.min_panel_area_fraction < self.max_panel_area_fraction < 1:
            raise ValueError('invalid panel area bounds')
        if not 0 < self.min_panel_aspect < self.max_panel_aspect:
            raise ValueError('invalid panel aspect bounds')
        if not 0 < self.center_half_width < 0.4 or not 0 < self.center_half_height < 0.4:
            raise ValueError('center half sizes must be in (0, 0.4)')
        if not 0 < self.min_panel_dark_fraction <= 1 or not 0 < self.panel_dark_threshold < 256:
            raise ValueError('invalid dark-panel threshold')
        if not 0 < self.minimum_battery_remaining < 1:
            raise ValueError('invalid battery threshold')
        if not isinstance(self.acquisition_frames, int) or self.acquisition_frames < 2:
            raise ValueError('acquisition_frames must be an integer >= 2')
        if not isinstance(self.panel_dark_threshold, int):
            raise ValueError('panel_dark_threshold must be integer')
        if self.maximum_tick_gap_s < 2 / self.control_hz:
            raise ValueError('maximum_tick_gap_s must allow two control periods')


def load_config(path):
    values = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(values, dict):
        raise ValueError('configuration must be a mapping')
    return Config(**values)
