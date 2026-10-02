"""Build a metric multi-panel map from laptop perception and MAVROS pose."""

from collections import deque
import math
import time
from typing import Optional

from da_daka_control.panel_mapping import (
    camera_surface_distance,
    CameraGroundModel,
    nearest_timestamped_sample,
    PanelMapBuilder,
    PanelObservation,
    project_panel_observation_attitude,
    quaternion_tilt_rad,
)
from da_daka_interfaces.msg import (
    PanelMap,
    PanelTarget as PanelTargetMessage,
    PerceptionResult,
)
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class PanelSurveyNode(Node):
    """Fuse all panel candidates seen during the 3 m survey window."""

    def __init__(self) -> None:
        super().__init__('panel_survey')
        self._declare_parameters()
        self._load_parameters()

        self._camera = CameraGroundModel(
            self._footprint_width_at_1m_m,
            self._footprint_height_at_1m_m,
            self._image_x_positive_is_left,
            self._image_y_positive_is_forward,
        )
        self._builder = PanelMapBuilder(
            self._merge_radius_m,
            self._minimum_observations,
        )
        self._active = False
        self._finalized = False
        self._pose: Optional[PoseStamped] = None
        self._pose_time_s: Optional[float] = None
        self._distance_m: Optional[float] = None
        self._range_time_s: Optional[float] = None
        self._pose_history = deque()
        self._range_history = deque()
        self._last_result_key = None
        self._last_state = ''

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_publisher = self.create_publisher(
            PanelMap,
            self._map_topic,
            latched_qos,
        )
        self._state_publisher = self.create_publisher(
            String,
            self._state_topic,
            latched_qos,
        )
        self.create_subscription(
            PerceptionResult,
            self._perception_topic,
            self._perception_callback,
            latched_qos,
        )
        self.create_subscription(
            PoseStamped,
            self._pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            self._range_topic,
            self._range_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self._active_topic,
            self._active_callback,
            latched_qos,
        )
        self.create_service(Trigger, self._reset_service, self._reset_callback)
        self.create_service(
            Trigger,
            self._finalize_service,
            self._finalize_callback,
        )
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._publish_map,
        )
        self._publish_state('IDLE')
        self._publish_map()

    def _declare_parameters(self) -> None:
        self.declare_parameter('perception_topic', '/ai/perception')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('range_topic', '/distance/filtered')
        self.declare_parameter('active_topic', '/panel_survey/active')
        self.declare_parameter('map_topic', '/panel_survey/map')
        self.declare_parameter('state_topic', '/panel_survey/state')
        self.declare_parameter('reset_service', '/panel_survey/reset')
        self.declare_parameter('finalize_service', '/panel_survey/finalize')
        self.declare_parameter('footprint_width_at_1m_m', 1.30)
        self.declare_parameter('footprint_height_at_1m_m', 0.73)
        self.declare_parameter('image_x_positive_is_left', False)
        self.declare_parameter('image_y_positive_is_forward', False)
        self.declare_parameter('camera_mount_roll_deg', 0.0)
        self.declare_parameter('camera_mount_pitch_deg', 0.0)
        self.declare_parameter('camera_mount_yaw_deg', 0.0)
        self.declare_parameter('camera_offset_forward_m', 0.0)
        self.declare_parameter('camera_offset_left_m', 0.0)
        self.declare_parameter('camera_offset_up_m', 0.0)
        self.declare_parameter('camera_height_above_range_sensor_m', 0.0)
        self.declare_parameter('merge_radius_m', 0.45)
        self.declare_parameter('minimum_observations', 3)
        self.declare_parameter('minimum_panel_confidence', 0.55)
        self.declare_parameter('maximum_survey_tilt_deg', 7.0)
        self.declare_parameter('input_timeout_s', 0.5)
        self.declare_parameter('capture_sync_tolerance_s', 0.15)
        self.declare_parameter('input_history_s', 2.0)
        self.declare_parameter('publish_rate_hz', 5.0)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._perception_topic = str(value('perception_topic'))
        self._pose_topic = str(value('pose_topic'))
        self._range_topic = str(value('range_topic'))
        self._active_topic = str(value('active_topic'))
        self._map_topic = str(value('map_topic'))
        self._state_topic = str(value('state_topic'))
        self._reset_service = str(value('reset_service'))
        self._finalize_service = str(value('finalize_service'))
        self._footprint_width_at_1m_m = float(
            value('footprint_width_at_1m_m')
        )
        self._footprint_height_at_1m_m = float(
            value('footprint_height_at_1m_m')
        )
        self._image_x_positive_is_left = bool(
            value('image_x_positive_is_left')
        )
        self._image_y_positive_is_forward = bool(
            value('image_y_positive_is_forward')
        )
        self._camera_mount_rpy_rad = tuple(
            math.radians(float(value(name)))
            for name in (
                'camera_mount_roll_deg',
                'camera_mount_pitch_deg',
                'camera_mount_yaw_deg',
            )
        )
        self._camera_offset_body_m = tuple(
            float(value(name))
            for name in (
                'camera_offset_forward_m',
                'camera_offset_left_m',
                'camera_offset_up_m',
            )
        )
        self._camera_height_above_range_sensor_m = float(
            value('camera_height_above_range_sensor_m')
        )
        self._merge_radius_m = float(value('merge_radius_m'))
        self._minimum_observations = int(value('minimum_observations'))
        self._minimum_panel_confidence = float(
            value('minimum_panel_confidence')
        )
        self._maximum_survey_tilt_rad = math.radians(
            float(value('maximum_survey_tilt_deg'))
        )
        self._input_timeout_s = float(value('input_timeout_s'))
        self._capture_sync_tolerance_s = float(
            value('capture_sync_tolerance_s')
        )
        self._input_history_s = float(value('input_history_s'))
        self._publish_rate_hz = float(value('publish_rate_hz'))
        self._sync_accepted = 0
        self._sync_rejected = 0
        self._worst_sync_skew_s = 0.0
        self._last_sync_log_s = 0.0
        if not 0.0 <= self._minimum_panel_confidence <= 1.0:
            raise ValueError('minimum_panel_confidence must be within [0, 1]')
        if not 0.0 < self._maximum_survey_tilt_rad <= math.pi / 2.0:
            raise ValueError('maximum_survey_tilt_deg must be within (0, 90]')
        if min(
            self._input_timeout_s,
            self._capture_sync_tolerance_s,
            self._input_history_s,
            self._publish_rate_hz,
        ) <= 0.0:
            raise ValueError('survey timeouts/rates must be positive')
        if self._input_history_s <= self._capture_sync_tolerance_s:
            raise ValueError('input history must exceed sync tolerance')
        if not math.isfinite(self._camera_height_above_range_sensor_m):
            raise ValueError('camera/range height calibration must be finite')

    def _pose_callback(self, message: PoseStamped) -> None:
        self._pose = message
        self._pose_time_s = time.monotonic()
        timestamp_ns = self._stamp_ns(message.header.stamp)
        self._pose_history.append((timestamp_ns, message))
        self._prune_history(self._pose_history, timestamp_ns)

    def _range_callback(self, message: Range) -> None:
        distance_m = float(message.range)
        if not math.isfinite(distance_m):
            return
        if distance_m < float(message.min_range):
            return
        if float(message.max_range) > 0.0 and distance_m > float(message.max_range):
            return
        self._distance_m = distance_m
        self._range_time_s = time.monotonic()
        timestamp_ns = self._stamp_ns(message.header.stamp)
        self._range_history.append((timestamp_ns, distance_m))
        self._prune_history(self._range_history, timestamp_ns)

    def _active_callback(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self._active:
            self._builder.reset()
            self._last_result_key = None
            self._finalized = False
            self._publish_state('SURVEYING')
        elif not requested and self._active:
            self._finalized = True
            self._publish_state('FINALIZED')
            self._publish_map()
        self._active = requested

    def _perception_callback(self, message: PerceptionResult) -> None:
        if not self._active or self._finalized or not bool(message.valid):
            return
        if str(message.mode).lower() != 'survey':
            return
        result_key = (str(message.session_id), int(message.sequence))
        if result_key == self._last_result_key:
            return
        now_s = time.monotonic()
        if not self._inputs_fresh(now_s):
            self._publish_state('WAITING_FOR_POSE_OR_RANGE')
            return
        self._last_result_key = result_key
        capture_ns = self._stamp_ns(message.header.stamp)
        pose_message = nearest_timestamped_sample(
            self._pose_history,
            capture_ns,
            self._capture_sync_tolerance_s,
        )
        distance_m = nearest_timestamped_sample(
            self._range_history,
            capture_ns,
            self._capture_sync_tolerance_s,
        )
        if pose_message is None or distance_m is None:
            self._sync_rejected += 1
            skew_s = self._nearest_skew_s(capture_ns)
            if skew_s is not None:
                self._worst_sync_skew_s = max(self._worst_sync_skew_s, skew_s)
            self._log_sync_rejection(skew_s)
            # A skew far past the tolerance is a clock disagreement between
            # the camera and this node, not a tolerance that needs widening.
            # Name the two apart so the ground check is unambiguous.
            if skew_s is not None and skew_s > 4.0 * self._capture_sync_tolerance_s:
                self._publish_state('CAPTURE_SYNC_CLOCK_SKEW')
            else:
                self._publish_state('WAITING_FOR_CAPTURE_SYNC')
            return
        self._sync_accepted += 1
        pose = pose_message.pose
        try:
            orientation = (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
            if quaternion_tilt_rad(orientation) > self._maximum_survey_tilt_rad:
                self._publish_state('TILT_REJECTED')
                return
            for detected in message.panels:
                confidence = float(detected.confidence)
                if confidence < self._minimum_panel_confidence:
                    continue
                projected = project_panel_observation_attitude(
                    PanelObservation(
                        center_x_norm=float(detected.center_x_norm),
                        center_y_norm=float(detected.center_y_norm),
                        width_norm=float(detected.width_norm),
                        height_norm=float(detected.height_norm),
                        confidence=confidence,
                    ),
                    self._camera,
                    vehicle_east_m=float(pose.position.x),
                    vehicle_north_m=float(pose.position.y),
                    vehicle_up_m=float(pose.position.z),
                    vehicle_quaternion_xyzw=orientation,
                    measured_center_distance_m=camera_surface_distance(
                        distance_m,
                        self._camera_height_above_range_sensor_m,
                    ),
                    camera_mount_rpy_rad=self._camera_mount_rpy_rad,
                    camera_offset_body_m=self._camera_offset_body_m,
                )
                self._builder.observe(projected)
        except ValueError as exc:
            self.get_logger().warning(f'Rejected survey result: {exc}')
            return
        self._publish_state('SURVEYING')
        self._publish_map()

    def _nearest_skew_s(self, capture_ns: int) -> float | None:
        """Return how far the closest pose/range sample missed the capture."""
        deltas = [
            min(abs(timestamp_ns - capture_ns) for timestamp_ns, _ in history)
            for history in (self._pose_history, self._range_history)
            if history
        ]
        if not deltas:
            return None
        return max(deltas) / 1e9

    def _log_sync_rejection(self, skew_s: float | None) -> None:
        """Report the rejection rate slowly enough to stay readable."""
        now_s = time.monotonic()
        if now_s - self._last_sync_log_s < 2.0:
            return
        self._last_sync_log_s = now_s
        total = self._sync_accepted + self._sync_rejected
        rate = self._sync_rejected / total if total else 0.0
        measured = 'unknown' if skew_s is None else f'{skew_s:.3f} s'
        self.get_logger().warning(
            'Capture sync rejected a survey result: skew '
            f'{measured} exceeds tolerance '
            f'{self._capture_sync_tolerance_s:.3f} s '
            f'(rejected {self._sync_rejected}/{total}, {rate:.0%}; '
            f'worst {self._worst_sync_skew_s:.3f} s)'
        )

    def _stamp_ns(self, stamp) -> int:
        """Convert one ROS timestamp, falling back only for zero stamps."""
        timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return timestamp_ns if timestamp_ns > 0 else self.get_clock().now().nanoseconds

    def _prune_history(self, history, newest_timestamp_ns: int) -> None:
        """Keep enough pose/range history to match delayed camera results."""
        cutoff_ns = newest_timestamp_ns - int(self._input_history_s * 1e9)
        while history and history[0][0] < cutoff_ns:
            history.popleft()

    def _inputs_fresh(self, now_s: float) -> bool:
        return (
            self._pose is not None
            and self._pose_time_s is not None
            and now_s - self._pose_time_s <= self._input_timeout_s
            and self._distance_m is not None
            and self._range_time_s is not None
            and now_s - self._range_time_s <= self._input_timeout_s
        )

    def _reset_callback(self, _request, response):
        self._builder.reset()
        self._last_result_key = None
        self._finalized = False
        response.success = True
        response.message = 'panel survey reset'
        self._publish_state('SURVEYING' if self._active else 'IDLE')
        self._publish_map()
        return response

    def _finalize_callback(self, _request, response):
        targets = self._builder.targets()
        self._active = False
        self._finalized = True
        response.success = bool(targets)
        response.message = (
            f'finalized {len(targets)} panels'
            if targets
            else 'no stable panel target was observed'
        )
        self._publish_state('FINALIZED' if targets else 'EMPTY')
        self._publish_map()
        return response

    def _publish_map(self) -> None:
        message = PanelMap()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.finalized = self._finalized
        for target in self._builder.targets():
            item = PanelTargetMessage()
            item.panel_id = target.panel_id
            item.east_m = target.east_m
            item.north_m = target.north_m
            item.width_m = target.width_m
            item.height_m = target.height_m
            item.confidence = target.confidence
            item.observation_count = target.observation_count
            message.panels.append(item)
        self._map_publisher.publish(message)

    def _publish_state(self, state: str) -> None:
        if state == self._last_state:
            return
        self._state_publisher.publish(String(data=state))
        self._last_state = state
        self.get_logger().info(f'Panel survey state={state}')


def main(args=None) -> None:
    """Run the panel survey node."""
    rclpy.init(args=args)
    node = PanelSurveyNode()
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
