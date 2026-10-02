import math


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def wrapped_angle_error(target_rad, current_rad):
    if not math.isfinite(target_rad) or not math.isfinite(current_rad):
        raise ValueError('yaw angles must be finite')
    return (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_rate_command(*, target_rad, current_rad, kp, maximum_rate_rad_s):
    # Preserved from the supplied distance_controller.py, including wrap/limit.
    if kp <= 0.0 or maximum_rate_rad_s <= 0.0:
        raise ValueError('yaw hold gains and limits must be positive')
    return clamp(kp * wrapped_angle_error(target_rad, current_rad),
                 -maximum_rate_rad_s, maximum_rate_rad_s)


def body_to_enu(forward, left, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * forward - s * left, s * forward + c * left


def slew_xy(current, target, maximum_delta):
    dx, dy = target[0] - current[0], target[1] - current[1]
    length = math.hypot(dx, dy)
    scale = min(1.0, maximum_delta / length) if length else 1.0
    return current[0] + scale * dx, current[1] + scale * dy


def quaternion_yaw(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.05:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def yaw_reset_innovation(previous, current, body_yaw_rate, dt):
    return wrapped_angle_error(wrapped_angle_error(current, previous), body_yaw_rate * dt)
