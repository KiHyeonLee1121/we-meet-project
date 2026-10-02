"""Camera-to-nozzle ground-point calibration helpers."""

from dataclasses import dataclass
import math

from da_daka_control.panel_mapping import body_offset_to_enu, CameraGroundModel


@dataclass(frozen=True)
class NozzleImageTarget:
    """Desired image point that places the nozzle over a ground target."""

    x_norm: float
    y_norm: float
    inside_safe_frame: bool


def quaternion_yaw_rad(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """Return ENU yaw from a finite, non-zero quaternion."""
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError('attitude quaternion must be finite')
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        raise ValueError('attitude quaternion norm is zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def body_velocity_to_enu(
    forward_mps: float,
    left_mps: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """Rotate a body-FLU visual correction into MAVROS local ENU."""
    return body_offset_to_enu(forward_mps, left_mps, yaw_rad)


def nozzle_image_target(
    camera: CameraGroundModel,
    *,
    camera_to_nozzle_forward_m: float,
    camera_to_nozzle_left_m: float,
    distance_m: float,
    safe_margin_norm: float = 0.05,
) -> NozzleImageTarget:
    """Convert the physical camera/nozzle offset to an image servo target."""
    values = (
        camera_to_nozzle_forward_m,
        camera_to_nozzle_left_m,
        distance_m,
        safe_margin_norm,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nozzle calibration inputs must be finite')
    if distance_m <= 0.0:
        raise ValueError('distance_m must be positive')
    if not 0.0 <= safe_margin_norm < 0.5:
        raise ValueError('safe_margin_norm must be within [0, 0.5)')
    x_norm, y_norm = camera.body_to_normalized(
        camera_to_nozzle_forward_m,
        camera_to_nozzle_left_m,
        distance_m,
    )
    inside = (
        safe_margin_norm <= x_norm <= 1.0 - safe_margin_norm
        and safe_margin_norm <= y_norm <= 1.0 - safe_margin_norm
    )
    return NozzleImageTarget(x_norm, y_norm, inside)


def compute_image_velocity(
    *,
    observed_x_norm: float,
    observed_y_norm: float,
    target_x_norm: float,
    target_y_norm: float,
    deadband_norm: float,
    gain_mps_per_norm: float,
    maximum_speed_mps: float,
    x_velocity_axis: str = 'y',
    y_velocity_axis: str = 'x',
    invert_x: bool = True,
    invert_y: bool = True,
) -> tuple[float, float, bool]:
    """Return body-FLU forward/left corrections toward the nozzle target."""
    values = (
        observed_x_norm,
        observed_y_norm,
        target_x_norm,
        target_y_norm,
        deadband_norm,
        gain_mps_per_norm,
        maximum_speed_mps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('visual alignment inputs must be finite')
    if not all(0.0 <= value <= 1.0 for value in values[:4]):
        raise ValueError('image coordinates must be within [0, 1]')
    if not 0.0 <= deadband_norm < 0.5:
        raise ValueError('deadband_norm must be within [0, 0.5)')
    if gain_mps_per_norm < 0.0 or maximum_speed_mps <= 0.0:
        raise ValueError('visual gain/limit values are invalid')
    if {x_velocity_axis, y_velocity_axis} != {'x', 'y'}:
        raise ValueError('image axes must map to distinct x/y velocity axes')

    error_x = observed_x_norm - target_x_norm
    error_y = observed_y_norm - target_y_norm
    aligned = abs(error_x) <= deadband_norm and abs(error_y) <= deadband_norm
    if aligned:
        return 0.0, 0.0, True

    image_x_cmd = 0.0
    image_y_cmd = 0.0
    if abs(error_x) > deadband_norm:
        image_x_cmd = max(
            -maximum_speed_mps,
            min(maximum_speed_mps, gain_mps_per_norm * error_x),
        )
    if abs(error_y) > deadband_norm:
        image_y_cmd = max(
            -maximum_speed_mps,
            min(maximum_speed_mps, gain_mps_per_norm * error_y),
        )
    if invert_x:
        image_x_cmd *= -1.0
    if invert_y:
        image_y_cmd *= -1.0
    commands = {x_velocity_axis: image_x_cmd, y_velocity_axis: image_y_cmd}
    return commands['x'], commands['y'], False
