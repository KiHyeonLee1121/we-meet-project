"""Camera worker attached to the same ROS/FC adapter as v1."""
from dataclasses import asdict
import math
import queue
import threading
import time
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from we_meet_flight_core.node import FlightNode as CoreFlightNode, run_node
from .config import load_config
from .image_io import image_to_bgr
from .mission import Mission, Sensors
from .vision import PanelDetector, TargetTracker


class FlightNode(CoreFlightNode):
    package_name = 'we_meet_flight'
    api_prefix = '/test_flying_v2'
    default_profile = 'panel_approach.yaml'
    load_settings = staticmethod(load_config)
    mission_type = Mission
    sensor_type = Sensors

    def extra_parameters(self):
        return {'camera_axes_verified': False, 'image_topic': '/camera/image_raw'}

    def extra_metadata(self):
        self.axes_verified = self.get_parameter('camera_axes_verified').value
        return {'camera_axes_verified': self.axes_verified}

    def create_extra_interfaces(self):
        self.image_queue = queue.Queue(maxsize=1)
        self.worker_stop = threading.Event()
        self.tracker = TargetTracker(self.c)
        self.tracker_lock = threading.Lock()
        if self.c.vision_enabled:
            self.create_subscription(Image, self.get_parameter('image_topic').value,
                                     self.on_image, qos_profile_sensor_data)
            self.worker = threading.Thread(target=self.vision_worker, daemon=True)
            self.worker.start()

    def check_extra_start(self):
        if not self.dry and self.c.vision_enabled and not self.axes_verified:
            raise ValueError('live camera trial requires verified camera axes')

    def extra_snapshot(self, now):
        self.s.camera_age = now-self.receipts.get('camera', -math.inf)

    def on_image(self, msg):
        now = self.receipt('camera_transport', msg, self.c.max_frame_age_s)
        if now is None:
            return
        item = (msg, self.receipts['camera_transport'], self.source_stamps['camera_transport'])
        try:
            self.image_queue.put_nowait(item)
        except queue.Full:
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.image_queue.put_nowait(item)
            except queue.Full:
                pass

    def vision_worker(self):
        detector = PanelDetector(self.c)
        while not self.worker_stop.is_set():
            try:
                msg, received, stamp = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if time.monotonic()-received > self.c.camera_timeout_s:
                    continue
                candidates = detector.detect(image_to_bgr(msg))
                with self.tracker_lock:
                    obs = self.tracker.update(candidates, received, stamp)
                    if obs is not None:
                        self.s.observation = obs
                        self.receipts['camera'] = received
                self.journal.append({'kind': 'vision', 'time_s': received, 'source_stamp_s': stamp,
                                     'observation': asdict(obs) if obs else None,
                                     'candidates': [asdict(d) for d in candidates]})
            except Exception as exc:
                self.journal.append({'kind': 'vision_error', 'time_s': received, 'error': str(exc)})

    def destroy_extra(self):
        self.worker_stop.set()
        if hasattr(self, 'worker'):
            self.worker.join(timeout=0.5)


def main(args=None):
    run_node(FlightNode, args)
