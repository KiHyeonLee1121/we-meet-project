"""Send requested survey/clean mode and panel ID from Pi to laptop."""

import json
import socket
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, String


class PerceptionControlSenderNode(Node):
    """Publish a small idempotent UDP control heartbeat to the laptop."""

    def __init__(self) -> None:
        """Create a new restart-safe control heartbeat session."""
        super().__init__('perception_control_sender')
        self.declare_parameter('laptop_ip', '127.0.0.1')
        self.declare_parameter('laptop_port', 5006)
        self.declare_parameter('source_id', 'pi5-01')
        self.declare_parameter('send_rate_hz', 5.0)
        self._laptop_ip = str(self.get_parameter('laptop_ip').value)
        self._laptop_port = int(self.get_parameter('laptop_port').value)
        self._source_id = str(self.get_parameter('source_id').value)
        send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        if not 1 <= self._laptop_port <= 65535:
            raise ValueError('laptop_port must be within [1, 65535]')
        if send_rate_hz <= 0.0 or not self._source_id:
            raise ValueError('control sender rate/source is invalid')
        self._mode = 'idle'
        self._panel_id = -1
        self._sequence = 0
        self._session_id = str(uuid.uuid4())
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(
            String, '/ai/requested_mode', self._mode_callback, 10
        )
        self.create_subscription(
            Int32,
            '/autonomous_cleaning/current_panel_id',
            self._panel_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / send_rate_hz, self._send)

    def _mode_callback(self, message: String) -> None:
        mode = str(message.data).lower()
        if mode in {'idle', 'survey', 'clean'}:
            self._mode = mode

    def _panel_callback(self, message: Int32) -> None:
        self._panel_id = int(message.data)

    def _send(self) -> None:
        self._sequence += 1
        payload = json.dumps(
            {
                'protocol_version': 1,
                'source_id': self._source_id,
                'session_id': self._session_id,
                'sequence': self._sequence,
                'send_timestamp_ns': time.time_ns(),
                'mode': self._mode,
                'active_panel_id': self._panel_id,
            },
            separators=(',', ':'),
        ).encode('utf-8')
        try:
            self._socket.sendto(payload, (self._laptop_ip, self._laptop_port))
        except OSError as exc:
            self.get_logger().error(
                f'AI control UDP send failed: {exc}',
                throttle_duration_sec=2.0,
            )

    def destroy_node(self) -> bool:
        """Close the UDP socket before destroying the ROS node."""
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the Pi-to-laptop perception control sender."""
    rclpy.init(args=args)
    node = PerceptionControlSenderNode()
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
