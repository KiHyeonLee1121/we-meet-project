"""Metric panel projection and multi-frame map construction."""

from dataclasses import dataclass
import math
from typing import Iterable, TypeVar


_Sample = TypeVar('_Sample')


def _finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def nearest_timestamped_sample(
    samples: Iterable[tuple[int, _Sample]],
    target_timestamp_ns: int,
    maximum_delta_s: float,
) -> _Sample | None:
    """Return the sample closest to one camera capture timestamp."""
    if target_timestamp_ns <= 0:
        raise ValueError('target timestamp must be positive')
    if not math.isfinite(maximum_delta_s) or maximum_delta_s <= 0.0:
        raise ValueError('maximum timestamp delta must be finite and positive')
    nearest = None
    nearest_delta_ns = math.inf
    for timestamp_ns, sample in samples:
        if timestamp_ns <= 0:
            continue
        delta_ns = abs(timestamp_ns - target_timestamp_ns)
        if delta_ns < nearest_delta_ns:
            nearest = sample
            nearest_delta_ns = delta_ns
    if nearest is None or nearest_delta_ns > maximum_delta_s * 1e9:
        return None
    return nearest


def camera_surface_distance(
    range_sensor_distance_m: float,
    camera_height_above_sensor_m: float,
) -> float:
    """Convert a range-sensor surface distance to camera optical distance."""
    if not _finite(range_sensor_distance_m, camera_height_above_sensor_m):
        raise ValueError('camera/range height inputs must be finite')
    distance_m = range_sensor_distance_m + camera_height_above_sensor_m
    if distance_m <= 0.0:
        raise ValueError('camera surface distance must be positive')
    return distance_m


@dataclass(frozen=True)
class CameraGroundModel:
    """Map a downward camera's normalized image coordinates to body FLU."""

    footprint_width_at_1m_m: float
    footprint_height_at_1m_m: float
    image_x_positive_is_left: bool = False
    image_y_positive_is_forward: bool = False

    def __post_init__(self) -> None:
        if not _finite(
            self.footprint_width_at_1m_m,
            self.footprint_height_at_1m_m,
        ):
            raise ValueError('camera footprint values must be finite')
        if min(
            self.footprint_width_at_1m_m,
            self.footprint_height_at_1m_m,
        ) <= 0.0:
            raise ValueError('camera footprint values must be positive')

    def footprint(self, distance_m: float) -> tuple[float, float]:
        """Return ground footprint width and height at a measured distance."""
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            raise ValueError('distance_m must be finite and positive')
        return (
            self.footprint_width_at_1m_m * distance_m,
            self.footprint_height_at_1m_m * distance_m,
        )

    def normalized_to_body(
        self,
        x_norm: float,
        y_norm: float,
        distance_m: float,
    ) -> tuple[float, float]:
        """Return body-forward and body-left metres for an image point."""
        if not _finite(x_norm, y_norm):
            raise ValueError('normalized coordinates must be finite')
        if not 0.0 <= x_norm <= 1.0 or not 0.0 <= y_norm <= 1.0:
            raise ValueError('normalized coordinates must be within [0, 1]')
        width_m, height_m = self.footprint(distance_m)
        left_sign = 1.0 if self.image_x_positive_is_left else -1.0
        forward_sign = 1.0 if self.image_y_positive_is_forward else -1.0
        left_m = left_sign * (x_norm - 0.5) * width_m
        forward_m = forward_sign * (y_norm - 0.5) * height_m
        return forward_m, left_m

    def body_to_normalized(
        self,
        forward_m: float,
        left_m: float,
        distance_m: float,
    ) -> tuple[float, float]:
        """Return the normalized image point for a body-frame ground point."""
        if not _finite(forward_m, left_m):
            raise ValueError('body offsets must be finite')
        width_m, height_m = self.footprint(distance_m)
        left_sign = 1.0 if self.image_x_positive_is_left else -1.0
        forward_sign = 1.0 if self.image_y_positive_is_forward else -1.0
        x_norm = 0.5 + left_m / (left_sign * width_m)
        y_norm = 0.5 + forward_m / (forward_sign * height_m)
        return x_norm, y_norm


@dataclass(frozen=True)
class PanelObservation:
    """One panel rectangle reported by the laptop perception worker."""

    center_x_norm: float
    center_y_norm: float
    width_norm: float
    height_norm: float
    confidence: float

    def __post_init__(self) -> None:
        values = (
            self.center_x_norm,
            self.center_y_norm,
            self.width_norm,
            self.height_norm,
            self.confidence,
        )
        if not _finite(*values):
            raise ValueError('panel observation values must be finite')
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError('panel observation values must be within [0, 1]')
        if self.width_norm <= 0.0 or self.height_norm <= 0.0:
            raise ValueError('panel rectangle dimensions must be positive')
        if self.confidence <= 0.0:
            raise ValueError('panel confidence must be positive')


@dataclass(frozen=True)
class MetricPanelObservation:
    """One panel observation projected into local ENU metres."""

    east_m: float
    north_m: float
    width_m: float
    height_m: float
    confidence: float


@dataclass(frozen=True)
class PanelTarget:
    """Stable panel target produced from repeated metric observations."""

    panel_id: int
    east_m: float
    north_m: float
    width_m: float
    height_m: float
    confidence: float
    observation_count: int


def body_offset_to_enu(
    forward_m: float,
    left_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    """Rotate a body FLU ground offset into local ENU."""
    if not _finite(forward_m, left_m, yaw_rad):
        raise ValueError('body-to-ENU inputs must be finite')
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        cosine * forward_m - sine * left_m,
        sine * forward_m + cosine * left_m,
    )


def _rotate_rpy(
    vector: tuple[float, float, float],
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    """Rotate a vector with intrinsic body X/Y/Z roll, pitch and yaw."""
    if not _finite(*vector, roll_rad, pitch_rad, yaw_rad):
        raise ValueError('rotation inputs must be finite')
    x, y, z = vector
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    x, y, z = x, cr * y - sr * z, sr * y + cr * z
    x, y, z = cp * x + sp * z, y, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


def rotate_body_to_enu(
    vector: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Rotate one body-FLU vector into ROS local ENU using a quaternion."""
    if not _finite(*vector, *quaternion_xyzw):
        raise ValueError('body-to-ENU quaternion inputs must be finite')
    qx, qy, qz, qw = quaternion_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError('vehicle quaternion norm is zero')
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def quaternion_tilt_rad(
    quaternion_xyzw: tuple[float, float, float, float],
) -> float:
    """Return vehicle tilt from local-up independent of yaw."""
    if not _finite(*quaternion_xyzw):
        raise ValueError('tilt quaternion values must be finite')
    qx, qy, qz, qw = quaternion_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError('tilt quaternion norm is zero')
    qx, qy = qx / norm, qy / norm
    body_up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.acos(max(-1.0, min(1.0, body_up_z)))


def _camera_ray_body(
    camera: CameraGroundModel,
    x_norm: float,
    y_norm: float,
    camera_mount_rpy_rad: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Build an unnormalised body-FLU ray for a downward camera pixel."""
    forward_m, left_m = camera.normalized_to_body(x_norm, y_norm, 1.0)
    return _rotate_rpy(
        (forward_m, left_m, -1.0),
        *camera_mount_rpy_rad,
    )


def project_panel_observation_attitude(
    observation: PanelObservation,
    camera: CameraGroundModel,
    *,
    vehicle_east_m: float,
    vehicle_north_m: float,
    vehicle_up_m: float,
    vehicle_quaternion_xyzw: tuple[float, float, float, float],
    measured_center_distance_m: float,
    camera_mount_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0),
    camera_offset_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MetricPanelObservation:
    """
    Project a panel to metric ENU using full attitude and a ground plane.

    The range is the slant distance along the calibrated camera centre ray.
    That ray fixes a local horizontal ground plane; panel centre and edge rays
    are intersected with it to correct real roll and pitch during the survey.
    """
    if not _finite(
        vehicle_east_m,
        vehicle_north_m,
        vehicle_up_m,
        measured_center_distance_m,
        *camera_mount_rpy_rad,
        *camera_offset_body_m,
    ):
        raise ValueError('attitude projection inputs must be finite')
    if measured_center_distance_m <= 0.0:
        raise ValueError('measured_center_distance_m must be positive')

    origin_offset_enu = rotate_body_to_enu(
        camera_offset_body_m,
        vehicle_quaternion_xyzw,
    )
    origin = (
        vehicle_east_m + origin_offset_enu[0],
        vehicle_north_m + origin_offset_enu[1],
        vehicle_up_m + origin_offset_enu[2],
    )
    center_ray_enu = rotate_body_to_enu(
        _camera_ray_body(camera, 0.5, 0.5, camera_mount_rpy_rad),
        vehicle_quaternion_xyzw,
    )
    center_norm = math.sqrt(sum(value * value for value in center_ray_enu))
    if center_norm <= 1e-9:
        raise ValueError('camera centre ray has zero length')
    center_unit = tuple(value / center_norm for value in center_ray_enu)
    if center_unit[2] >= -1e-4:
        raise ValueError('camera centre ray does not point toward the ground')
    ground_up_m = origin[2] + center_unit[2] * measured_center_distance_m

    def intersect(x_norm: float, y_norm: float) -> tuple[float, float]:
        ray_enu = rotate_body_to_enu(
            _camera_ray_body(camera, x_norm, y_norm, camera_mount_rpy_rad),
            vehicle_quaternion_xyzw,
        )
        if ray_enu[2] >= -1e-4:
            raise ValueError('panel ray does not intersect the ground below')
        scale = (ground_up_m - origin[2]) / ray_enu[2]
        if scale <= 0.0 or not math.isfinite(scale):
            raise ValueError('panel ray has no forward ground intersection')
        return origin[0] + scale * ray_enu[0], origin[1] + scale * ray_enu[1]

    half_width = observation.width_norm / 2.0
    half_height = observation.height_norm / 2.0
    left_x = observation.center_x_norm - half_width
    right_x = observation.center_x_norm + half_width
    top_y = observation.center_y_norm - half_height
    bottom_y = observation.center_y_norm + half_height
    if not (
        0.0 <= left_x <= right_x <= 1.0
        and 0.0 <= top_y <= bottom_y <= 1.0
    ):
        raise ValueError('panel rectangle extends outside the image')
    centre = intersect(observation.center_x_norm, observation.center_y_norm)
    left = intersect(left_x, observation.center_y_norm)
    right = intersect(right_x, observation.center_y_norm)
    top = intersect(observation.center_x_norm, top_y)
    bottom = intersect(observation.center_x_norm, bottom_y)
    return MetricPanelObservation(
        east_m=centre[0],
        north_m=centre[1],
        width_m=math.hypot(right[0] - left[0], right[1] - left[1]),
        height_m=math.hypot(bottom[0] - top[0], bottom[1] - top[1]),
        confidence=observation.confidence,
    )


def project_panel_observation(
    observation: PanelObservation,
    camera: CameraGroundModel,
    *,
    vehicle_east_m: float,
    vehicle_north_m: float,
    vehicle_yaw_rad: float,
    distance_m: float,
) -> MetricPanelObservation:
    """Project a normalized panel rectangle into local ENU metres."""
    if not _finite(vehicle_east_m, vehicle_north_m, vehicle_yaw_rad):
        raise ValueError('vehicle pose must be finite')
    forward_m, left_m = camera.normalized_to_body(
        observation.center_x_norm,
        observation.center_y_norm,
        distance_m,
    )
    east_offset_m, north_offset_m = body_offset_to_enu(
        forward_m,
        left_m,
        vehicle_yaw_rad,
    )
    footprint_width_m, footprint_height_m = camera.footprint(distance_m)
    return MetricPanelObservation(
        east_m=vehicle_east_m + east_offset_m,
        north_m=vehicle_north_m + north_offset_m,
        width_m=observation.width_norm * footprint_width_m,
        height_m=observation.height_norm * footprint_height_m,
        confidence=observation.confidence,
    )


@dataclass
class _PanelCluster:
    panel_id: int
    east_weighted: float
    north_weighted: float
    width_weighted: float
    height_weighted: float
    total_weight: float
    confidence_sum: float
    count: int

    @property
    def east_m(self) -> float:
        return self.east_weighted / self.total_weight

    @property
    def north_m(self) -> float:
        return self.north_weighted / self.total_weight

    def add(self, observation: MetricPanelObservation) -> None:
        weight = max(observation.confidence, 1e-6)
        self.east_weighted += observation.east_m * weight
        self.north_weighted += observation.north_m * weight
        self.width_weighted += observation.width_m * weight
        self.height_weighted += observation.height_m * weight
        self.total_weight += weight
        self.confidence_sum += observation.confidence
        self.count += 1


class PanelMapBuilder:
    """Fuse repeated projected observations into stable panel targets."""

    def __init__(self, merge_radius_m: float, minimum_observations: int) -> None:
        if not math.isfinite(merge_radius_m) or merge_radius_m <= 0.0:
            raise ValueError('merge_radius_m must be finite and positive')
        if minimum_observations <= 0:
            raise ValueError('minimum_observations must be positive')
        self.merge_radius_m = merge_radius_m
        self.minimum_observations = minimum_observations
        self.reset()

    def reset(self) -> None:
        """Discard all survey observations."""
        self._clusters: list[_PanelCluster] = []
        self._next_panel_id = 1

    def observe(self, observation: MetricPanelObservation) -> int:
        """Merge one observation and return its stable panel ID."""
        values = (
            observation.east_m,
            observation.north_m,
            observation.width_m,
            observation.height_m,
            observation.confidence,
        )
        if not _finite(*values):
            raise ValueError('metric panel observation must be finite')
        if min(
            observation.width_m,
            observation.height_m,
            observation.confidence,
        ) <= 0.0:
            raise ValueError('metric panel dimensions/confidence must be positive')

        nearest = None
        nearest_distance = math.inf
        for cluster in self._clusters:
            distance = math.hypot(
                observation.east_m - cluster.east_m,
                observation.north_m - cluster.north_m,
            )
            if distance < nearest_distance:
                nearest = cluster
                nearest_distance = distance
        if nearest is not None and nearest_distance <= self.merge_radius_m:
            nearest.add(observation)
            return nearest.panel_id

        weight = max(observation.confidence, 1e-6)
        cluster = _PanelCluster(
            panel_id=self._next_panel_id,
            east_weighted=observation.east_m * weight,
            north_weighted=observation.north_m * weight,
            width_weighted=observation.width_m * weight,
            height_weighted=observation.height_m * weight,
            total_weight=weight,
            confidence_sum=observation.confidence,
            count=1,
        )
        self._clusters.append(cluster)
        self._next_panel_id += 1
        return cluster.panel_id

    def targets(self) -> tuple[PanelTarget, ...]:
        """Return only clusters supported by enough independent frames."""
        targets = []
        for cluster in self._clusters:
            if cluster.count < self.minimum_observations:
                continue
            targets.append(
                PanelTarget(
                    panel_id=cluster.panel_id,
                    east_m=cluster.east_m,
                    north_m=cluster.north_m,
                    width_m=cluster.width_weighted / cluster.total_weight,
                    height_m=cluster.height_weighted / cluster.total_weight,
                    confidence=cluster.confidence_sum / cluster.count,
                    observation_count=cluster.count,
                )
            )
        return tuple(sorted(targets, key=lambda target: target.panel_id))
