"""Independently request landing when climb from the launch point is excessive."""

import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String


class AltitudeGuardCore:
    """Track a ground reference and latch excessive relative climb."""

    def __init__(self, maximum_climb_m: float) -> None:
        if maximum_climb_m <= 0.0:
            raise ValueError('maximum_climb_m must be greater than zero')
        self.maximum_climb_m = maximum_climb_m
        self.ground_z_m: Optional[float] = None
        self.launch_z_m: Optional[float] = None
        self.triggered = False

    def observe_ground(self, local_z_m: float) -> None:
        """Update the launch reference only while disarmed and landed."""
        if math.isfinite(local_z_m):
            self.ground_z_m = float(local_z_m)

    def arm(self) -> bool:
        """Latch the most recent ground sample when the vehicle arms."""
        if self.ground_z_m is None:
            return False
        self.launch_z_m = self.ground_z_m
        self.triggered = False
        return True

    def disarm(self) -> None:
        """Clear the flight latch so a new ground reference can be captured."""
        self.launch_z_m = None
        self.triggered = False

    def update(self, local_z_m: float) -> Optional[float]:
        """Return relative climb and latch once the configured limit is met."""
        if self.launch_z_m is None or not math.isfinite(local_z_m):
            return None
        climb_m = float(local_z_m) - self.launch_z_m
        if climb_m >= self.maximum_climb_m:
            self.triggered = True
        return climb_m


class AltitudeGuardNode(Node):
    """Request AUTO.LAND independently of the normal mission sequence."""

    LANDED_STATE_ON_GROUND = 1

    def __init__(self) -> None:
        super().__init__('altitude_guard')
        self.declare_parameter('maximum_climb_m', 5.0)
        self.declare_parameter('pose_timeout_s', 0.5)
        self.declare_parameter('check_rate_hz', 20.0)
        self.declare_parameter('land_mode', 'AUTO.LAND')
        self.declare_parameter('request_retry_interval_s', 0.5)
        self.declare_parameter('land_on_pose_timeout', True)
        self.declare_parameter('land_on_missing_launch_reference', True)

        maximum_climb_m = float(
            self.get_parameter('maximum_climb_m').value
        )
        self._pose_timeout_s = float(
            self.get_parameter('pose_timeout_s').value
        )
        check_rate_hz = float(self.get_parameter('check_rate_hz').value)
        self._land_mode = str(self.get_parameter('land_mode').value)
        self._retry_interval_s = float(
            self.get_parameter('request_retry_interval_s').value
        )
        self._land_on_pose_timeout = bool(
            self.get_parameter('land_on_pose_timeout').value
        )
        self._land_on_missing_reference = bool(
            self.get_parameter('land_on_missing_launch_reference').value
        )
        if self._pose_timeout_s <= 0.0:
            raise ValueError('pose_timeout_s must be greater than zero')
        if check_rate_hz <= 0.0:
            raise ValueError('check_rate_hz must be greater than zero')
        if self._retry_interval_s <= 0.0:
            raise ValueError(
                'request_retry_interval_s must be greater than zero'
            )

        self._guard = AltitudeGuardCore(maximum_climb_m)
        self._connected = False
        self._armed = False
        self._mode = ''
        self._landed_state: Optional[int] = None
        self._latest_z_m: Optional[float] = None
        self._pose_time_s: Optional[float] = None
        self._landing_latched = False
        self._landing_reason = ''
        self._last_request_s = -math.inf
        self._request_pending = False

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._triggered_publisher = self.create_publisher(
            Bool,
            '/altitude_guard/triggered',
            latched_qos,
        )
        self._reason_publisher = self.create_publisher(
            String,
            '/altitude_guard/reason',
            latched_qos,
        )
        self._climb_publisher = self.create_publisher(
            Float32,
            '/altitude_guard/climb_m',
            10,
        )
        self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            10,
        )
        self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self._extended_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self._mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode',
        )
        self._timer = self.create_timer(1.0 / check_rate_hz, self._tick)
        self._publish_status()
        self.get_logger().info(
            'Altitude guard ready; '
            f'landing at {maximum_climb_m:.2f} m above launch point'
        )

    def _state_callback(self, message: State) -> None:
        was_armed = self._armed
        self._connected = bool(message.connected)
        self._armed = bool(message.armed)
        self._mode = str(message.mode)

        if self._armed and not was_armed:
            if self._guard.arm():
                self.get_logger().info(
                    f'Launch reference latched at local Z='
                    f'{self._guard.launch_z_m:.3f} m'
                )
            elif self._land_on_missing_reference:
                self._trigger_land(
                    'vehicle armed without a valid launch altitude reference'
                )
        elif was_armed and not self._armed:
            self._guard.disarm()
            self._landing_latched = False
            self._landing_reason = ''
            self._request_pending = False
            self._publish_status()

    def _extended_state_callback(self, message: ExtendedState) -> None:
        self._landed_state = int(message.landed_state)
        self._capture_ground_reference()

    def _pose_callback(self, message: PoseStamped) -> None:
        local_z_m = float(message.pose.position.z)
        if not math.isfinite(local_z_m):
            return
        self._latest_z_m = local_z_m
        self._pose_time_s = time.monotonic()
        self._capture_ground_reference()
        if not self._armed:
            return

        climb_m = self._guard.update(local_z_m)
        if climb_m is None:
            return
        self._climb_publisher.publish(Float32(data=climb_m))
        if self._guard.triggered:
            self._trigger_land(
                f'climb limit exceeded: {climb_m:.2f} m >= '
                f'{self._guard.maximum_climb_m:.2f} m'
            )

    def _capture_ground_reference(self) -> None:
        if (
            not self._armed
            and self._landed_state == self.LANDED_STATE_ON_GROUND
            and self._latest_z_m is not None
        ):
            self._guard.observe_ground(self._latest_z_m)

    def _tick(self) -> None:
        now_s = time.monotonic()
        if self._armed and self._land_on_pose_timeout:
            pose_stale = (
                self._pose_time_s is None
                or now_s - self._pose_time_s > self._pose_timeout_s
            )
            if pose_stale:
                self._trigger_land('local altitude telemetry timed out')

        if not self._landing_latched or not self._armed:
            return
        if self._mode == self._land_mode:
            return
        if not self._connected:
            self.get_logger().error(
                'Landing is latched but MAVROS is disconnected; '
                'PX4 onboard failsafe must recover the vehicle',
                throttle_duration_sec=2.0,
            )
            return
        if self._request_pending:
            return
        if now_s - self._last_request_s < self._retry_interval_s:
            return
        if not self._mode_client.service_is_ready():
            self.get_logger().error(
                'Landing is latched but /mavros/set_mode is unavailable',
                throttle_duration_sec=2.0,
            )
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self._land_mode
        self._request_pending = True
        self._last_request_s = now_s
        future = self._mode_client.call_async(request)
        future.add_done_callback(self._land_request_done)
        self.get_logger().error(
            f'Requesting {self._land_mode}: {self._landing_reason}'
        )

    def _trigger_land(self, reason: str) -> None:
        if self._landing_latched:
            return
        self._landing_latched = True
        self._landing_reason = reason
        self._publish_status()
        self.get_logger().fatal(f'ALTITUDE GUARD TRIGGERED: {reason}')

    def _land_request_done(self, future) -> None:
        self._request_pending = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - transport failure
            self.get_logger().error(f'AUTO.LAND request failed: {error}')
            return
        if not response.mode_sent:
            self.get_logger().error(
                'PX4 rejected AUTO.LAND request; retry remains active'
            )

    def _publish_status(self) -> None:
        self._triggered_publisher.publish(
            Bool(data=self._landing_latched)
        )
        self._reason_publisher.publish(String(data=self._landing_reason))


def main(args=None) -> None:
    """Run the independent launch-point altitude guard."""
    rclpy.init(args=args)
    node = AltitudeGuardNode()
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
