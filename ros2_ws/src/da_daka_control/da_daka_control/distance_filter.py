"""Reject invalid range readings and smooth valid distance measurements."""

from collections import deque
import math
import statistics
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class DistanceFilterCore:
    """Median filter followed by a moving-average filter."""

    def __init__(self, median_window_size: int, average_window_size: int) -> None:
        if median_window_size < 1 or median_window_size % 2 == 0:
            raise ValueError('median_window_size must be a positive odd number')
        if average_window_size < 1:
            raise ValueError('average_window_size must be positive')

        self._raw_window: Deque[float] = deque(maxlen=median_window_size)
        self._median_window: Deque[float] = deque(maxlen=average_window_size)

    def update(self, value: float) -> float:
        """Add one valid sample and return the filtered distance."""
        self._raw_window.append(value)
        median_value = statistics.median(self._raw_window)
        self._median_window.append(median_value)
        return statistics.fmean(self._median_window)


def valid_measurement(message: Range) -> Tuple[bool, str]:
    """Validate a range value against finiteness and message limits."""
    if not math.isfinite(message.range):
        return False, 'non-finite value'
    if message.range < message.min_range:
        return False, 'below sensor minimum'
    if message.range > message.max_range:
        return False, 'above sensor maximum'
    return True, ''


class DistanceFilterNode(Node):
    """Filter raw downward range data and preserve Range metadata."""

    def __init__(self) -> None:
        super().__init__('distance_filter')
        self.declare_parameter('input_topic', '/distance/raw')
        self.declare_parameter('output_topic', '/distance/filtered')
        self.declare_parameter('median_window_size', 5)
        self.declare_parameter('average_window_size', 5)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        median_size = int(self.get_parameter('median_window_size').value)
        average_size = int(self.get_parameter('average_window_size').value)

        self._filter = DistanceFilterCore(median_size, average_size)
        self._last_warning_ns: Optional[int] = None
        self._publisher = self.create_publisher(
            Range,
            output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Range,
            input_topic,
            self._range_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}; '
            f'median={median_size}, moving_average={average_size}'
        )

    def _warn_invalid(self, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_warning_ns is None or now_ns - self._last_warning_ns >= 2_000_000_000:
            self.get_logger().warning(f'Discarding invalid distance sample: {reason}')
            self._last_warning_ns = now_ns

    def _range_callback(self, message: Range) -> None:
        is_valid, reason = valid_measurement(message)
        if not is_valid:
            self._warn_invalid(reason)
            return

        output = Range()
        output.header = message.header
        output.radiation_type = message.radiation_type
        output.field_of_view = message.field_of_view
        output.min_range = message.min_range
        output.max_range = message.max_range
        output.range = self._filter.update(float(message.range))
        self._publisher.publish(output)


def main(args=None) -> None:
    """Run the distance filter node."""
    rclpy.init(args=args)
    node = DistanceFilterNode()
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
