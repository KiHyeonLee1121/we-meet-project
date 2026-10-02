"""Shared flight defaults, plus camera settings and legacy YAML migration."""
from dataclasses import dataclass, fields
import math
from pathlib import Path
import yaml
from we_meet_flight_core.config import Config as CoreConfig


@dataclass(frozen=True)
class Config(CoreConfig):
    vision_enabled: bool = True
    camera_timeout_s: float = 0.3
    target_loss_s: float = 0.6
    align_timeout_s: float = 30.0
    visual_hold_timeout_s: float = 15.0
    visual_gain_mps: float = 0.35
    visual_max_speed_mps: float = 0.12
    center_half_width: float = 0.15
    center_half_height: float = 0.15
    # Image x/y -> body FLU forward/left; verify on the installed camera.
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
        super().__post_init__()
        if not isinstance(self.vision_enabled, bool):
            raise ValueError('vision_enabled must be boolean')
        if self.vision_enabled and not self.advance_enabled:
            raise ValueError('vision requires advance_enabled')
        shared = {f.name for f in fields(CoreConfig)}
        for f in fields(Config):
            if f.name in shared or f.name in {'vision_enabled', 'image_to_body'}:
                continue
            value = getattr(self, f.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{f.name} must be finite positive numeric')
        matrix = self.image_to_body
        if not isinstance(matrix, (tuple, list)) or len(matrix) != 2 or any(not isinstance(r, (tuple, list)) or len(r) != 2 for r in matrix):
            raise ValueError('image_to_body must be a 2x2 matrix')
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for row in matrix for v in row):
            raise ValueError('image_to_body must be finite numeric')
        if any(abs(sum(v*v for v in row)-1) > 1e-6 for row in matrix) or abs(sum(matrix[0][i]*matrix[1][i] for i in range(2))) > 1e-6:
            raise ValueError('image_to_body must have orthonormal rows')
        if not 0 < self.min_panel_area_fraction < self.max_panel_area_fraction < 1:
            raise ValueError('invalid panel area bounds')
        if not 0 < self.min_panel_aspect < self.max_panel_aspect:
            raise ValueError('invalid panel aspect bounds')
        if not 0 < self.center_half_width < 0.4 or not 0 < self.center_half_height < 0.4:
            raise ValueError('invalid centre region')
        if not 0 < self.min_panel_dark_fraction <= 1 or not 0 < self.panel_dark_threshold < 256:
            raise ValueError('invalid dark threshold')
        if not isinstance(self.panel_dark_threshold, int) or not isinstance(self.acquisition_frames, int) or self.acquisition_frames < 2:
            raise ValueError('threshold/frame count must be integers; acquisition >= 2')
        if self.visual_hold_timeout_s <= self.zero_velocity_hold_s:
            raise ValueError('visual hold timeout must exceed hold duration')


LEGACY_KEYS = {'approach_enabled': 'advance_enabled', 'target_range_m': 'target_height_m',
               'approach_distance_m': 'commanded_target_distance_m',
               'cruise_speed_mps': 'forward_speed_mps', 'hover_s': 'zero_velocity_hold_s'}


def load_config(path):
    values = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(values, dict):
        raise ValueError('configuration must be a mapping')
    for old, new in LEGACY_KEYS.items():
        if old in values:
            if new in values:
                raise ValueError(f'conflicting configuration keys: {old}, {new}')
            values[new] = values.pop(old)
    return Config(**values)
