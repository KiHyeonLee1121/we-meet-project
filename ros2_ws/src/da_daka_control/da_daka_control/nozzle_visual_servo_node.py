"""Align detected dirt to the physical spray-nozzle image point."""

import math
import time
from typing import Optional

from da_daka_control.nozzle_alignment import (
    body_velocity_to_enu,
    compute_image_velocity,
    nozzle_image_target,
    quaternion_yaw_rad,
)
from da_daka_control.panel_mapping import camera_surface_distance, CameraGroundModel
from da_daka_interfaces.msg import PerceptionResult
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class NozzleVisualServoNode(Node):
    """Publish bounded XY corrections without directly owning MAVROS."""

    def __init__(self) -> None:
        super().__init__('nozzle_visual_servo')
        self._declare_parameters()
        self._load_parameters()
        self._camera = CameraGroundModel(
            self._footprint_width_at_1m_m,
            self._footprint_height_at_1m_m,
            self._image_x_positive_is_left,
            self._image_y_positive_is_forward,
        )
        self._healthy = False
        self._result: Optional[PerceptionResult] = None
        self._result_time_s: Optional[float] = None
        self._distance_m: Optional[float] = None
        self._range_time_s: Optional[float] = None
        self._yaw_rad: Optional[float] = None
        self._pose_time_s: Optional[float] = None
        self._last_aligned: Optional[bool] = None
        self._last_valid: Optional[bool] = None
        self._last_visible: Optional[bool] = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._command_publisher = self.create_publisher(
            TwistStamped,
            self._command_topic,
            qos_profile_sensor_data,
        )
        self._aligned_publisher = self.create_publisher(
            Bool, self._aligned_topic, latched_qos
        )
        self._valid_publisher = self.create_publisher(
            Bool, self._valid_topic, latched_qos
        )
        self._visible_publisher = self.create_publisher(
            Bool, self._visible_topic, latched_qos
        )
        self.create_subscription(
            PerceptionResult,
            self._result_topic,
            self._result_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            self._health_topic,
            self._health_callback,
            latched_qos,
        )
        self.create_subscription(
            Range,
            self._range_topic,
            self._range_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self._pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)
        self._publish_states(False, False, False)

    def _declare_parameters(self) -> None:
        self.declare_parameter('result_topic', '/ai/perception')
        self.declare_parameter('health_topic', '/ai/health')
        self.declare_parameter('range_topic', '/distance/filtered')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('command_topic', '/visual_servo/cmd_vel_xy')
        self.declare_parameter('aligned_topic', '/visual_servo/aligned')
        self.declare_parameter('valid_topic', '/visual_servo/target_valid')
        self.declare_parameter('visible_topic', '/visual_servo/panel_visible')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('input_timeout_s', 0.5)
        self.declare_parameter('footprint_width_at_1m_m', 1.30)
        self.declare_parameter('footprint_height_at_1m_m', 0.73)
        self.declare_parameter('image_x_positive_is_left', False)
        self.declare_parameter('image_y_positive_is_forward', False)
        self.declare_parameter('camera_to_nozzle_forward_m', 0.0)
        self.declare_parameter('camera_to_nozzle_left_m', 0.0)
        self.declare_parameter('camera_height_above_lidar_m', 0.0)
        self.declare_parameter('safe_frame_margin_norm', 0.05)
        self.declare_parameter('deadband_norm', 0.035)
        self.declare_parameter('gain_mps_per_norm', 0.35)
        self.declare_parameter('maximum_speed_mps', 0.12)
        self.declare_parameter('image_x_velocity_axis', 'y')
        self.declare_parameter('image_y_velocity_axis', 'x')
        self.declare_parameter('invert_image_x', True)
        self.declare_parameter('invert_image_y', True)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._result_topic = str(value('result_topic'))
        self._health_topic = str(value('health_topic'))
        self._range_topic = str(value('range_topic'))
        self._pose_topic = str(value('pose_topic'))
        self._command_topic = str(value('command_topic'))
        self._aligned_topic = str(value('aligned_topic'))
        self._valid_topic = str(value('valid_topic'))
        self._visible_topic = str(value('visible_topic'))
        self._frame_id = str(value('frame_id'))
        self._rate_hz = float(value('rate_hz'))
        self._input_timeout_s = float(value('input_timeout_s'))
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
        self._nozzle_forward_m = float(value('camera_to_nozzle_forward_m'))
        self._nozzle_left_m = float(value('camera_to_nozzle_left_m'))
        self._camera_height_above_lidar_m = float(
            value('camera_height_above_lidar_m')
        )
        self._safe_margin_norm = float(value('safe_frame_margin_norm'))
        self._deadband_norm = float(value('deadband_norm'))
        self._gain = float(value('gain_mps_per_norm'))
        self._maximum_speed_mps = float(value('maximum_speed_mps'))
        self._image_x_axis = str(value('image_x_velocity_axis')).lower()
        self._image_y_axis = str(value('image_y_velocity_axis')).lower()
        self._invert_image_x = bool(value('invert_image_x'))
        self._invert_image_y = bool(value('invert_image_y'))
        if min(self._rate_hz, self._input_timeout_s) <= 0.0:
            raise ValueError('visual servo rates/timeouts must be positive')
        if self._frame_id != 'map':
            raise ValueError('visual servo output frame_id must be map/ENU')
        if not all(
            math.isfinite(value)
            for value in (
                self._nozzle_forward_m,
                self._nozzle_left_m,
                self._camera_height_above_lidar_m,
            )
        ):
            raise ValueError('visual servo mount calibration must be finite')

    def _result_callback(self, message: PerceptionResult) -> None:
        self._result = message
        self._result_time_s = time.monotonic()

    def _health_callback(self, message: Bool) -> None:
        self._healthy = bool(message.data)

    def _range_callback(self, message: Range) -> None:
        distance_m = float(message.range)
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            return
        self._distance_m = distance_m
        self._range_time_s = time.monotonic()

    def _pose_callback(self, message: PoseStamped) -> None:
        orientation = message.pose.orientation
        try:
            self._yaw_rad = quaternion_yaw_rad(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        except ValueError:
            return
        self._pose_time_s = time.monotonic()

    def _tick(self) -> None:
        now_s = time.monotonic()
        fresh = (
            self._healthy
            and self._result is not None
            and self._result_time_s is not None
            and now_s - self._result_time_s <= self._input_timeout_s
            and self._distance_m is not None
            and self._range_time_s is not None
            and now_s - self._range_time_s <= self._input_timeout_s
            and self._yaw_rad is not None
            and self._pose_time_s is not None
            and now_s - self._pose_time_s <= self._input_timeout_s
        )
        visible = bool(fresh and self._result.panel_visible)
        target_valid = bool(
            visible
            and self._result.valid
            and self._result.dirt_found
            and 0.0 <= float(self._result.dirt_centroid_x_norm) <= 1.0
            and 0.0 <= float(self._result.dirt_centroid_y_norm) <= 1.0
        )
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self._frame_id
        aligned = False
        if target_valid:
            try:
                target = nozzle_image_target(
                    self._camera,
                    camera_to_nozzle_forward_m=self._nozzle_forward_m,
                    camera_to_nozzle_left_m=self._nozzle_left_m,
                    distance_m=camera_surface_distance(
                        self._distance_m,
                        self._camera_height_above_lidar_m,
                    ),
                    safe_margin_norm=self._safe_margin_norm,
                )
                if target.inside_safe_frame:
                    body_forward_mps, body_left_mps, aligned = compute_image_velocity(
                        observed_x_norm=float(
                            self._result.dirt_centroid_x_norm
                        ),
                        observed_y_norm=float(
                            self._result.dirt_centroid_y_norm
                        ),
                        target_x_norm=target.x_norm,
                        target_y_norm=target.y_norm,
                        deadband_norm=self._deadband_norm,
                        gain_mps_per_norm=self._gain,
                        maximum_speed_mps=self._maximum_speed_mps,
                        x_velocity_axis=self._image_x_axis,
                        y_velocity_axis=self._image_y_axis,
                        invert_x=self._invert_image_x,
                        invert_y=self._invert_image_y,
                    )
                    east_mps, north_mps = body_velocity_to_enu(
                        body_forward_mps,
                        body_left_mps,
                        self._yaw_rad,
                    )
                    command.twist.linear.x = east_mps
                    command.twist.linear.y = north_mps
                else:
                    target_valid = False
            except ValueError as exc:
                target_valid = False
                self.get_logger().error(
                    f'Unsafe nozzle calibration: {exc}',
                    throttle_duration_sec=2.0,
                )
        self._command_publisher.publish(command)
        self._publish_states(aligned, target_valid, visible)

    def _publish_states(
        self,
        aligned: bool,
        valid: bool,
        visible: bool,
    ) -> None:
        if aligned != self._last_aligned:
            self._aligned_publisher.publish(Bool(data=aligned))
            self._last_aligned = aligned
        if valid != self._last_valid:
            self._valid_publisher.publish(Bool(data=valid))
            self._last_valid = valid
        if visible != self._last_visible:
            self._visible_publisher.publish(Bool(data=visible))
            self._last_visible = visible


def main(args=None) -> None:
    """Run the nozzle-aware visual servo node."""
    rclpy.init(args=args)
    node = NozzleVisualServoNode()
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
