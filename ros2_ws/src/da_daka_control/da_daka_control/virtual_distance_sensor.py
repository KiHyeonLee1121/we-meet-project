"""Configurable virtual downward distance sensor for SITL development."""

import math
import random
import statistics
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


VALID_MODES = {'constant', 'approach', 'local_position', 'sine'}


def move_toward(current: float, target: float, maximum_step: float) -> float:
    """Move current toward target without overshooting."""
    if current < target:
        return min(current + maximum_step, target)
    return max(current - maximum_step, target)


class VirtualDistanceSensor(Node):
    """Publish simulated TF-Luna-like measurements as sensor_msgs/Range."""

    def __init__(self) -> None:
        super().__init__('virtual_distance_sensor')

        self.declare_parameter('topic', '/distance/raw')
        self.declare_parameter('frame_id', 'tf_luna_link')
        self.declare_parameter('local_position_topic', '/mavros/local_position/pose')
        self.declare_parameter('ground_reference_sample_count', 20)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('mode', 'constant')
        self.declare_parameter('initial_distance_m', 1.5)
        self.declare_parameter('target_distance_m', 1.0)
        self.declare_parameter('approach_rate_mps', 0.05)
        self.declare_parameter('sine_amplitude_m', 0.25)
        self.declare_parameter('sine_period_s', 8.0)
        self.declare_parameter('noise_stddev_m', 0.0)
        self.declare_parameter('spike_probability', 0.0)
        self.declare_parameter('spike_magnitude_m', 0.5)
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('field_of_view_rad', 0.035)
        self.declare_parameter('random_seed', 42)

        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        local_position_topic = str(
            self.get_parameter('local_position_topic').value
        )
        self.ground_reference_sample_count = int(
            self.get_parameter('ground_reference_sample_count').value
        )
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mode = str(self.get_parameter('mode').value)
        self.initial_distance_m = float(self.get_parameter('initial_distance_m').value)
        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.approach_rate_mps = float(self.get_parameter('approach_rate_mps').value)
        self.sine_amplitude_m = float(self.get_parameter('sine_amplitude_m').value)
        self.sine_period_s = float(self.get_parameter('sine_period_s').value)
        self.noise_stddev_m = float(self.get_parameter('noise_stddev_m').value)
        self.spike_probability = float(self.get_parameter('spike_probability').value)
        self.spike_magnitude_m = float(self.get_parameter('spike_magnitude_m').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.field_of_view_rad = float(self.get_parameter('field_of_view_rad').value)
        random_seed = int(self.get_parameter('random_seed').value)

        self._validate_parameters()
        self._rng = random.Random(random_seed)
        self._simulated_distance_m = self._clamp(self.initial_distance_m)
        self._elapsed_s = 0.0
        self._period_s = 1.0 / self.publish_rate_hz
        self._latest_local_z: Optional[float] = None
        self._ground_z: Optional[float] = None
        self._ground_samples = []

        self.publisher = self.create_publisher(
            Range,
            self.topic,
            qos_profile_sensor_data,
        )
        self.local_position_subscription = self.create_subscription(
            PoseStamped,
            local_position_topic,
            self._local_position_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self._period_s, self._publish_measurement)

        self.get_logger().info(
            f'Publishing {self.mode} virtual range on {self.topic} at '
            f'{self.publish_rate_hz:.1f} Hz; initial={self.initial_distance_m:.2f} m'
        )

    def _validate_parameters(self) -> None:
        if self.mode not in VALID_MODES:
            raise ValueError(f'mode must be one of {sorted(VALID_MODES)}, got {self.mode!r}')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        if self.ground_reference_sample_count < 1:
            raise ValueError('ground_reference_sample_count must be positive')
        if self.min_range_m < 0.0 or self.max_range_m <= self.min_range_m:
            raise ValueError('range limits must satisfy 0 <= min_range_m < max_range_m')
        if self.approach_rate_mps < 0.0:
            raise ValueError('approach_rate_mps cannot be negative')
        if self.sine_period_s <= 0.0:
            raise ValueError('sine_period_s must be greater than zero')
        if self.noise_stddev_m < 0.0:
            raise ValueError('noise_stddev_m cannot be negative')
        if not 0.0 <= self.spike_probability <= 1.0:
            raise ValueError('spike_probability must be between zero and one')

    def _clamp(self, value: float) -> float:
        return min(max(value, self.min_range_m), self.max_range_m)

    def _local_position_callback(self, message: PoseStamped) -> None:
        self._latest_local_z = float(message.pose.position.z)

    def _next_true_distance(self) -> Optional[float]:
        if self.mode == 'approach':
            step = self.approach_rate_mps * self._period_s
            self._simulated_distance_m = move_toward(
                self._simulated_distance_m,
                self._clamp(self.target_distance_m),
                step,
            )
        elif self.mode == 'local_position':
            if self._latest_local_z is None:
                return None
            if self._ground_z is None:
                self._ground_samples.append(self._latest_local_z)
                if len(self._ground_samples) < self.ground_reference_sample_count:
                    return None
                self._ground_z = statistics.median(self._ground_samples)
                self.get_logger().info(
                    f'Ground reference captured at local Z={self._ground_z:.3f} m'
                )
            self._simulated_distance_m = self._clamp(
                self._latest_local_z - self._ground_z
            )
        elif self.mode == 'sine':
            phase = 2.0 * math.pi * self._elapsed_s / self.sine_period_s
            self._simulated_distance_m = self._clamp(
                self.initial_distance_m + self.sine_amplitude_m * math.sin(phase)
            )

        self._elapsed_s += self._period_s
        return self._simulated_distance_m

    def _publish_measurement(self) -> None:
        measured = self._next_true_distance()
        if measured is None:
            return
        measured += self._rng.gauss(0.0, self.noise_stddev_m)

        if self._rng.random() < self.spike_probability:
            direction = -1.0 if self._rng.random() < 0.5 else 1.0
            measured += direction * self.spike_magnitude_m

        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.radiation_type = Range.INFRARED
        message.field_of_view = self.field_of_view_rad
        message.min_range = self.min_range_m
        message.max_range = self.max_range_m
        message.range = self._clamp(measured)
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the virtual range sensor node."""
    rclpy.init(args=args)
    node = VirtualDistanceSensor()
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
