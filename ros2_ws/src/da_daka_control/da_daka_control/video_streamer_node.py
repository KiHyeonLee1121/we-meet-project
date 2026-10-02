"""Supervise the Raspberry Pi 5 low-latency camera UDP stream."""

import subprocess
import time
from typing import Optional

from da_daka_control.video_streaming import build_rpicam_command
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class VideoStreamerNode(Node):
    """Start, monitor and stop one camera-owner process on the Pi."""

    def __init__(self) -> None:
        super().__init__('video_streamer')
        self.declare_parameter('enabled_on_startup', False)
        self.declare_parameter('executable', 'rpicam-vid')
        self.declare_parameter('laptop_ip', '127.0.0.1')
        self.declare_parameter('port', 5600)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 20)
        self.declare_parameter('bitrate_bps', 4000000)
        self.declare_parameter('shutter_us', 0)
        self.declare_parameter('gain', 0.0)
        self.declare_parameter('monitor_rate_hz', 2.0)
        self.declare_parameter('restart_delay_s', 2.0)
        self.declare_parameter('maximum_restart_attempts', 3)

        self._command = build_rpicam_command(
            executable=str(self.get_parameter('executable').value),
            laptop_ip=str(self.get_parameter('laptop_ip').value),
            port=int(self.get_parameter('port').value),
            width=int(self.get_parameter('width').value),
            height=int(self.get_parameter('height').value),
            framerate=int(self.get_parameter('framerate').value),
            bitrate_bps=int(self.get_parameter('bitrate_bps').value),
            shutter_us=int(self.get_parameter('shutter_us').value),
            gain=float(self.get_parameter('gain').value),
        )
        monitor_rate_hz = float(self.get_parameter('monitor_rate_hz').value)
        self._restart_delay_s = float(
            self.get_parameter('restart_delay_s').value
        )
        self._maximum_restarts = int(
            self.get_parameter('maximum_restart_attempts').value
        )
        if min(monitor_rate_hz, self._restart_delay_s) <= 0.0:
            raise ValueError('stream monitor timing must be positive')
        if self._maximum_restarts < 0:
            raise ValueError('maximum_restart_attempts cannot be negative')

        self._enabled = bool(self.get_parameter('enabled_on_startup').value)
        self._process: Optional[subprocess.Popen] = None
        self._restart_count = 0
        self._last_exit_s = -float('inf')
        self._last_healthy: Optional[bool] = None
        self._health_publisher = self.create_publisher(
            Bool, '/video_stream/healthy', 10
        )
        self._state_publisher = self.create_publisher(
            String, '/video_stream/state', 10
        )
        self.create_service(SetBool, '/video_stream/enable', self._enable)
        self._timer = self.create_timer(1.0 / monitor_rate_hz, self._tick)
        self._tick()

    def _enable(self, request, response):
        requested = bool(request.data)
        self._enabled = requested
        if not requested:
            self._stop_process()
            self._restart_count = 0
        response.success = True
        response.message = 'video stream enabled' if requested else 'disabled'
        self._tick()
        return response

    def _tick(self) -> None:
        now_s = time.monotonic()
        if not self._enabled:
            self._publish(False, 'DISABLED')
            return
        if self._process is not None and self._process.poll() is None:
            self._publish(True, 'STREAMING')
            return
        if self._process is not None:
            code = self._process.returncode
            self._process = None
            self._last_exit_s = now_s
            self._restart_count += 1
            self.get_logger().error(f'rpicam-vid exited with code {code}')
        if self._restart_count > self._maximum_restarts:
            self._enabled = False
            self._publish(False, 'FAILED_RESTART_LIMIT')
            return
        if now_s - self._last_exit_s < self._restart_delay_s:
            self._publish(False, 'WAITING_TO_RESTART')
            return
        try:
            self._process = subprocess.Popen(
                self._command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as exc:
            self._last_exit_s = now_s
            self._restart_count += 1
            self.get_logger().error(f'failed to start rpicam-vid: {exc}')
            self._publish(False, 'START_FAILED')
            return
        self._publish(True, 'STREAMING')

    def _publish(self, healthy: bool, state: str) -> None:
        if healthy != self._last_healthy:
            self._health_publisher.publish(Bool(data=healthy))
            self._last_healthy = healthy
        self._state_publisher.publish(String(data=state))

    def _stop_process(self) -> None:
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)
        self._process = None

    def destroy_node(self) -> bool:
        """Stop the camera process before destroying the ROS node."""
        self._stop_process()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the Pi camera-stream supervisor."""
    rclpy.init(args=args)
    node = VideoStreamerNode()
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
