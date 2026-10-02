"""Reconstructed Home-follow arithmetic recorded for flight 141.

This deliberately implements the historical reference motion. Home relatch is
not a local-frame reset; no modern origin/reset policy is substituted here.
"""
from dataclasses import dataclass
import math


def finite_xy(point):
    xy = tuple(float(v) for v in point[:2])
    if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
        raise ValueError('finite ENU XY is required')
    return xy


@dataclass
class LegacyHomeAnchor:
    request_xy: tuple[float, float]
    start_home_xy: tuple[float, float]
    maximum_shift_m: float = 10.0

    def __post_init__(self):
        self.request_xy = finite_xy(self.request_xy)
        self.start_home_xy = finite_xy(self.start_home_xy)
        if not math.isfinite(self.maximum_shift_m) or self.maximum_shift_m <= 0:
            raise ValueError('positive maximum Home shift is required')

    @property
    def offset_xy(self):
        return tuple(p - h for p, h in zip(self.request_xy, self.start_home_xy))

    def target_xy(self, current_home_xy):
        home = finite_xy(current_home_xy)
        shift = tuple(h - old for h, old in zip(home, self.start_home_xy))
        if math.hypot(*shift) > self.maximum_shift_m:
            raise ValueError('historical Home correction exceeds maximum shift')
        return tuple(h + off for h, off in zip(home, self.offset_xy))


def geodetic_offset_enu(lat, lon, home_lat, home_lon):
    """Short-distance tangent approximation used only for the start crosscheck."""
    values = (lat, lon, home_lat, home_lon)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('finite GPS/Home geodetic coordinates are required')
    if not (-90 <= lat <= 90 and -90 <= home_lat <= 90
            and -180 <= lon <= 180 and -180 <= home_lon <= 180):
        raise ValueError('GPS/Home coordinates out of range')
    radius = 6371000.0
    return (
        radius * math.cos(math.radians(home_lat)) * math.radians(lon - home_lon),
        radius * math.radians(lat - home_lat),
    )


def home_crosscheck_error(local_xy, home_xy, gps_latlon, home_latlon):
    offset = geodetic_offset_enu(*gps_latlon, *home_latlon)
    return math.hypot(*(p - h - g for p, h, g in zip(
        finite_xy(local_xy), finite_xy(home_xy), offset)))


def historical_panel_order(start_xy, targets, home_xy, recorded_ids=(3, 1, 2, 6)):
    """Reproduce the recorded ID order; use the archive planner for new IDs.

    The 141 field document and first low-altitude ULog target prove the first
    ID 3. The supplied snapshot instead plans (6,2,1,3) on the reference map.
    This explicit priority is a reconstruction, not an inferred exact planner.
    """
    from da_daka_control.route_planner import plan_panel_route
    ids = tuple(int(i) for i in recorded_ids)
    if any(i <= 0 for i in ids) or len(ids) != len(set(ids)):
        raise ValueError('historical panel priorities must be unique positive IDs')
    targets = tuple(targets)
    fallback = plan_panel_route(start_xy, targets, home_xy)
    known = {target.panel_id: target for target in fallback.targets}
    ordered = [i for i in ids if i in known]
    return tuple(ordered + [i for i in fallback.panel_ids if i not in ordered])
