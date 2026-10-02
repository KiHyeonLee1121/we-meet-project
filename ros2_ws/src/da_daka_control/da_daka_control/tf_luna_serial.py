"""Read TF-Luna binary frames and publish ROS 2 range measurements."""

from dataclasses import dataclass
import time
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
from serial import SerialException


FRAME_HEADER = b'\x59\x59'
FRAME_LENGTH = 9


@dataclass(frozen=True)
class TfLunaFrame:
    """One checksum-verified TF-Luna measurement."""

    distance_m: float
    strength: int
    temperature_c: float


class TfLunaParser:
    """Incrementally extract TF-Luna 9-byte binary frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.checksum_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> List[TfLunaFrame]:
        """Add bytes and return all complete, checksum-valid frames."""
        self._buffer.extend(data)
        frames: List[TfLunaFrame] = []

        while len(self._buffer) >= FRAME_LENGTH:
            header_index = self._buffer.find(FRAME_HEADER)
            if header_index < 0:
                keep = 1 if self._buffer[-1] == FRAME_HEADER[0] else 0
                discarded = len(self._buffer) - keep
                self.discarded_bytes += discarded
                del self._buffer[:discarded]
                break

            if header_index:
                self.discarded_bytes += header_index
                del self._buffer[:header_index]

            if len(self._buffer) < FRAME_LENGTH:
                break

            candidate = self._buffer[:FRAME_LENGTH]
            if sum(candidate[:8]) & 0xFF != candidate[8]:
                self.checksum_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            distance_cm = candidate[2] | candidate[3] << 8
            strength = candidate[4] | candidate[5] << 8
            temperature_raw = candidate[6] | candidate[7] << 8
            frames.append(
                TfLunaFrame(
                    distance_m=distance_cm / 100.0,
                    strength=strength,
                    temperature_c=temperature_raw / 8.0 - 256.0,
                )
            )
            del self._buffer[:FRAME_LENGTH]

        return frames


class TfLunaSerialNode(Node):
    """Own the TF-Luna serial port and publish sensor_msgs/Range."""

    def __init__(self) -> None:
        super().__init__('tf_luna_serial')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/'
            'usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0',
        )
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic', '/distance/raw')
        self.declare_parameter('frame_id', 'tf_luna_link')
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('field_of_view_rad', 0.035)
        self.declare_parameter('minimum_strength', 100)
        self.declare_parameter('poll_rate_hz', 200.0)
        self.declare_parameter('read_chunk_size', 512)
        self.declare_parameter('reconnect_interval_s', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.field_of_view_rad = float(
            self.get_parameter('field_of_view_rad').value
        )
        self.minimum_strength = int(self.get_parameter('minimum_strength').value)
        poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.reconnect_interval_s = float(
            self.get_parameter('reconnect_interval_s').value
        )

        if poll_rate_hz <= 0.0:
            raise ValueError('poll_rate_hz must be greater than zero')
        if not 0.0 <= self.min_range_m < self.max_range_m:
            raise ValueError('range limits must satisfy 0 <= min < max')
        if self.read_chunk_size <= 0:
            raise ValueError('read_chunk_size must be positive')

        self._publisher = self.create_publisher(Range, self.topic, 10)
        self._parser = TfLunaParser()
        self._serial = None
        self._next_connect_time = 0.0
        self._published_count = 0
        self._rejected_count = 0
        self._last_report_time = time.monotonic()
        self._timer = self.create_timer(1.0 / poll_rate_hz, self._poll)

        self.get_logger().info(
            f'TF-Luna configured for {self.port} at {self.baudrate} baud; '
            f'publishing {self.topic}'
        )

    def _connect(self) -> bool:
        now = time.monotonic()
        if self._serial is not None:
            return True
        if now < self._next_connect_time:
            return False

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0,
            )
            self._serial.reset_input_buffer()
            self.get_logger().info(f'Connected to TF-Luna on {self.port}')
            return True
        except (OSError, SerialException) as error:
            self._serial = None
            self._next_connect_time = now + self.reconnect_interval_s
            self.get_logger().warning(
                f'Cannot open TF-Luna serial port {self.port}: {error}',
                throttle_duration_sec=self.reconnect_interval_s,
            )
            return False

    def _disconnect(self, error: Exception) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        self._serial = None
        self._next_connect_time = time.monotonic() + self.reconnect_interval_s
        self.get_logger().error(f'TF-Luna serial connection lost: {error}')

    def _poll(self) -> None:
        if not self._connect():
            return

        try:
            available = max(1, int(self._serial.in_waiting))
            data = self._serial.read(min(available, self.read_chunk_size))
        except (OSError, SerialException) as error:
            self._disconnect(error)
            return

        for frame in self._parser.feed(data):
            if not self._valid(frame):
                self._rejected_count += 1
                continue
            self._publish(frame)

        self._report_health()

    def _valid(self, frame: TfLunaFrame) -> bool:
        return (
            self.min_range_m <= frame.distance_m <= self.max_range_m
            and frame.strength >= self.minimum_strength
        )

    def _publish(self, frame: TfLunaFrame) -> None:
        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.radiation_type = Range.INFRARED
        message.field_of_view = self.field_of_view_rad
        message.min_range = self.min_range_m
        message.max_range = self.max_range_m
        message.range = frame.distance_m
        self._publisher.publish(message)
        self._published_count += 1

    def _report_health(self) -> None:
        now = time.monotonic()
        if now - self._last_report_time < 10.0:
            return
        self.get_logger().info(
            f'TF-Luna health: published={self._published_count}, '
            f'rejected={self._rejected_count}, '
            f'checksum_errors={self._parser.checksum_errors}, '
            f'discarded_bytes={self._parser.discarded_bytes}'
        )
        self._last_report_time = now

    def destroy_node(self) -> bool:
        """Close the serial device before normal ROS shutdown."""
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the TF-Luna serial node."""
    rclpy.init(args=args)
    node = TfLunaSerialNode()
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
