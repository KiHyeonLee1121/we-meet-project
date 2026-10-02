"""Pure helpers for the localization-test survey route and timing."""

import math


def rectangular_scan_waypoints(
    launch_x_m: float,
    launch_y_m: float,
    width_m: float,
    depth_m: float,
    *,
    yaw_rad: float | None = None,
    forward_offset_m: float = 0.0,
    lateral_offset_m: float = 0.0,
) -> tuple[tuple[float, float], ...]:
    """Return four rectangle corners followed by the launch point."""
    # With yaw_rad the rectangle follows the vehicle heading rather than the
    # local North/East axes: width_m spans left-to-right across the airframe
    # and depth_m spans forward, matching how the panels sit beside and ahead
    # of the launch point. The offsets then shift its centre in body frame.
    # A world-aligned rectangle spends half of its legs behind the aircraft,
    # which wastes the scan whenever the panels sit ahead of the launch point.
    values = (
        launch_x_m,
        launch_y_m,
        width_m,
        depth_m,
        forward_offset_m,
        lateral_offset_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('survey route values must be finite')
    if width_m <= 0.0 or depth_m <= 0.0:
        raise ValueError('survey dimensions must be positive')
    if yaw_rad is not None and not math.isfinite(yaw_rad):
        raise ValueError('survey route values must be finite')

    half_width = width_m / 2.0
    half_depth = depth_m / 2.0
    corners_body = (
        (-half_width, -half_depth),
        (+half_width, -half_depth),
        (+half_width, +half_depth),
        (-half_width, +half_depth),
    )

    if yaw_rad is None:
        # Legacy world-aligned rectangle: offsets stay on the North/East axes.
        centre_x = launch_x_m + forward_offset_m
        centre_y = launch_y_m + lateral_offset_m
        return tuple(
            (centre_x + dx, centre_y + dy) for dx, dy in corners_body
        ) + ((launch_x_m, launch_y_m),)

    forward = (math.cos(yaw_rad), math.sin(yaw_rad))
    left = (-math.sin(yaw_rad), math.cos(yaw_rad))
    centre_x = (
        launch_x_m + forward_offset_m * forward[0] + lateral_offset_m * left[0]
    )
    centre_y = (
        launch_y_m + forward_offset_m * forward[1] + lateral_offset_m * left[1]
    )
    # dx spans the airframe left-right, dy spans forward.
    rotated = tuple(
        (
            centre_x + dx * left[0] + dy * forward[0],
            centre_y + dx * left[1] + dy * forward[1],
        )
        for dx, dy in corners_body
    )
    return rotated + ((launch_x_m, launch_y_m),)


def survey_timeout_elapsed_s(
    now_s: float,
    state_started_s: float,
    survey_started_s: float | None,
) -> float:
    """Measure setup timeout from state entry and scan timeout from scan start."""
    started_s = (
        survey_started_s if survey_started_s is not None else state_started_s
    )
    return max(0.0, now_s - started_s)
