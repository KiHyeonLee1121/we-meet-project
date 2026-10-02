"""Bridge validated laptop perception UDP results into typed ROS topics."""

import ipaddress
import socket
import time
from typing import Optional

from da_daka_control.perception_protocol import (
    decode_perception_packet,
    PerceptionPacket,
    PerceptionProtocolError,
    ProtocolConfig,
    SequenceTracker,
)
from da_daka_interfaces.msg import PanelDetection, PerceptionResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class PerceptionReceiverNode(Node):
    """Fail closed on malformed, stale, duplicated or missing AI results."""

    def __init__(self) -> None:
        super().__init__('perception_receiver')
        self._declare_parameters()
        self._load_parameters()
        self._tracker = SequenceTracker()
        self._config = ProtocolConfig(
            allowed_source_id=self._allowed_source_id,
            maximum_panels=self._maximum_panels,
            maximum_inference_time_ms=self._maximum_inference_time_ms,
        )
        self._last_receive_s: Optional[float] = None
        self._last_health: Optional[bool] = None
        self._accepted = 0
        self._rejected = 0
        self._last_rejection_log_s = -float('inf')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._result_publisher = self.create_publisher(
            PerceptionResult,
            self._result_topic,
            latched_qos,
        )
        self._health_publisher = self.create_publisher(
            Bool,
            self._health_topic,
            latched_qos,
        )
        self._state_publisher = self.create_publisher(
            String,
            self._state_topic,
            latched_qos,
        )
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._bind_address, self._port))
        self._socket.setblocking(False)
        self._timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll)
        self._publish_health(False)
        self._publish_state('NO_RESULT')
        self.get_logger().info(
            f'Perception UDP v2 listening on {self._bind_address}:{self._port}; '
            f'allowed source={self._allowed_source_id}; '
            f'allowed IP={self._allowed_remote_ip or "any"}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 5005)
        self.declare_parameter('allowed_source_id', 'laptop-ai-01')
        self.declare_parameter('allowed_remote_ip', '')
        self.declare_parameter('result_topic', '/ai/perception')
        self.declare_parameter('health_topic', '/ai/health')
        self.declare_parameter('state_topic', '/ai/receiver_state')
        self.declare_parameter('heartbeat_timeout_s', 1.0)
        self.declare_parameter('poll_rate_hz', 100.0)
        self.declare_parameter('maximum_packet_bytes', 65507)
        self.declare_parameter('maximum_panels', 32)
        self.declare_parameter('maximum_inference_time_ms', 1000.0)

    def _load_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self._bind_address = str(value('bind_address'))
        self._port = int(value('port'))
        self._allowed_source_id = str(value('allowed_source_id'))
        allowed_remote_ip = str(value('allowed_remote_ip')).strip()
        self._allowed_remote_ip = (
            str(ipaddress.ip_address(allowed_remote_ip))
            if allowed_remote_ip
            else ''
        )
        self._result_topic = str(value('result_topic'))
        self._health_topic = str(value('health_topic'))
        self._state_topic = str(value('state_topic'))
        self._heartbeat_timeout_s = float(value('heartbeat_timeout_s'))
        self._poll_rate_hz = float(value('poll_rate_hz'))
        self._maximum_packet_bytes = int(value('maximum_packet_bytes'))
        self._maximum_panels = int(value('maximum_panels'))
        self._maximum_inference_time_ms = float(
            value('maximum_inference_time_ms')
        )
        if not 1 <= self._port <= 65535:
            raise ValueError('port must be within [1, 65535]')
        if min(self._heartbeat_timeout_s, self._poll_rate_hz) <= 0.0:
            raise ValueError('receiver timeout/rate must be positive')
        if not 512 <= self._maximum_packet_bytes <= 65507:
            raise ValueError('maximum_packet_bytes is outside UDP limits')

    def _poll(self) -> None:
        newest = None
        while True:
            try:
                packet, address = self._socket.recvfrom(
                    self._maximum_packet_bytes
                )
            except BlockingIOError:
                break
            try:
                remote_ip = str(ipaddress.ip_address(address[0]))
                if (
                    self._allowed_remote_ip
                    and remote_ip != self._allowed_remote_ip
                ):
                    raise PerceptionProtocolError(
                        'remote_ip',
                        f'packet source {remote_ip} is not allowlisted',
                    )
                decoded = decode_perception_packet(packet, self._config)
                self._tracker.accept(decoded)
                newest = decoded
                self._accepted += 1
            except PerceptionProtocolError as exc:
                self._rejected += 1
                now_s = time.monotonic()
                if now_s - self._last_rejection_log_s >= 1.0:
                    self.get_logger().warning(
                        f'Rejected AI packet [{exc.code}]: {exc}'
                    )
                    self._last_rejection_log_s = now_s

        now_s = time.monotonic()
        if newest is not None:
            self._last_receive_s = now_s
            self._publish_result(newest)
        healthy = (
            self._last_receive_s is not None
            and now_s - self._last_receive_s <= self._heartbeat_timeout_s
        )
        self._publish_health(healthy)
        self._publish_state(
            f'HEALTHY accepted={self._accepted} rejected={self._rejected}'
            if healthy
            else f'STALE accepted={self._accepted} rejected={self._rejected}'
        )

    def _publish_result(self, packet: PerceptionPacket) -> None:
        message = PerceptionResult()
        now_ns = self.get_clock().now().nanoseconds
        capture_ns = int(packet.capture_timestamp_ns)
        message.header.stamp.sec = capture_ns // 1_000_000_000
        message.header.stamp.nanosec = capture_ns % 1_000_000_000
        message.header.frame_id = 'camera'
        message.protocol_version = packet.protocol_version
        message.source_id = packet.source_id
        message.session_id = packet.session_id
        message.frame_id = packet.frame_id
        message.sequence = packet.sequence
        message.mode = packet.mode
        message.valid = packet.valid
        message.panel_visible = packet.panel_visible
        message.active_panel_id = packet.active_panel_id
        message.dirt_found = packet.dirt_found
        message.dirt_centroid_x_norm = packet.dirt_centroid_x_norm
        message.dirt_centroid_y_norm = packet.dirt_centroid_y_norm
        message.dirt_bbox_x_norm = packet.dirt_bbox_x_norm
        message.dirt_bbox_y_norm = packet.dirt_bbox_y_norm
        message.dirt_bbox_w_norm = packet.dirt_bbox_w_norm
        message.dirt_bbox_h_norm = packet.dirt_bbox_h_norm
        message.dirt_confidence = packet.dirt_confidence
        message.inference_time_ms = packet.inference_time_ms
        message.result_age_s = max(0.0, (now_ns - capture_ns) / 1e9)
        message.invalid_reason = packet.invalid_reason
        message.model_name = packet.model_name
        for packet_panel in packet.panels:
            panel = PanelDetection()
            panel.candidate_id = packet_panel.candidate_id
            panel.center_x_norm = packet_panel.center_x_norm
            panel.center_y_norm = packet_panel.center_y_norm
            panel.width_norm = packet_panel.width_norm
            panel.height_norm = packet_panel.height_norm
            panel.confidence = packet_panel.confidence
            message.panels.append(panel)
        self._result_publisher.publish(message)

    def _publish_health(self, healthy: bool) -> None:
        if healthy == self._last_health:
            return
        self._health_publisher.publish(Bool(data=healthy))
        self._last_health = healthy

    def _publish_state(self, state: str) -> None:
        self._state_publisher.publish(String(data=state))

    def destroy_node(self) -> bool:
        """Close the result socket before destroying the ROS node."""
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the perception UDP receiver."""
    rclpy.init(args=args)
    node = PerceptionReceiverNode()
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
