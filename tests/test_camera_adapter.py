"""Camera adapter test doubles; never imports or contacts a real ROS/FC."""
from dataclasses import asdict
import importlib
import math
from pathlib import Path
import queue
import sys
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
import cv2  # Load native bindings before temporarily replacing ROS modules.
from unittest.mock import Mock, patch
from test_adapter import adapter
from test_mission import healthy
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight'))
from we_meet_flight.config import Config
from we_meet_flight.mission import Mission, Sensors

sensor_module = ModuleType('sensor_msgs.msg')
sensor_module.Image = object
qos_module = ModuleType('rclpy.qos')
qos_module.qos_profile_sensor_data = object()
with patch.dict(sys.modules, {'sensor_msgs.msg': sensor_module, 'rclpy.qos': qos_module,
                              'we_meet_flight_core.node': adapter}):
    visual_adapter = importlib.import_module('we_meet_flight.node')


class CameraAdapterTests(unittest.TestCase):
    def node(self):
        n = visual_adapter.FlightNode.__new__(visual_adapter.FlightNode)
        n.c, n.s = Config(), Sensors(**asdict(healthy()), camera_age=0)
        n.mission = Mission(n.c)
        n.dry, n.axes_verified = True, False
        n.receipts, n.source_stamps = {}, {}
        n.image_queue = queue.Queue(maxsize=1)
        n.worker_stop, n.tracker_lock = threading.Event(), threading.Lock()
        n.journal = NS(append=Mock())
        return n

    def test_live_start_camera_gate_preserved_with_shared_start_logic(self):
        n = self.node()
        n.dry = False
        n.lidar_verified = n.lidar_capable = n.settings_verified = n.frame_verified = True
        n.graph_fault = ''
        n.mode_client = n.arm_client = NS(service_is_ready=lambda: True)
        n.get_parameter = lambda name: NS(value=False)
        n.snapshot = lambda now: n.s
        n.stable_yaw = lambda now: 0
        result = n.on_start(None, NS())
        self.assertFalse(result.success)
        self.assertIn('camera axes', result.message)
        self.assertEqual(n.mission.state, 'IDLE')
        n.axes_verified = True
        result = n.on_start(None, NS())
        self.assertTrue(result.success)
        self.assertEqual(n.mission.state, 'PRESTREAM')

    def test_snapshot_camera_age_and_common_health_are_both_updated(self):
        n = self.node()
        n.receipts = {key: 10 for key in ['estimator', 'state', 'landed', 'yaw', 'imu', 'range', 'battery', 'camera']}
        s = n.snapshot(10.2)
        self.assertAlmostEqual(s.camera_age, 0.2)
        self.assertAlmostEqual(s.range_age, 0.2)
        self.assertAlmostEqual(s.estimator_age, 0.2)
        n.receipts.pop('camera')
        self.assertEqual(n.snapshot(10.3).camera_age, math.inf)

    def test_camera_queue_keeps_latest_accepted_source_frame(self):
        n = self.node()
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
        a = NS(header=NS(stamp=NS(sec=9, nanosec=900_000_000)))
        b = NS(header=NS(stamp=NS(sec=9, nanosec=950_000_000)))
        n.on_image(a); n.on_image(b); n.on_image(a)
        msg, received, stamp = n.image_queue.get_nowait()
        self.assertIs(msg, b)
        self.assertAlmostEqual(stamp, 9.95)
        self.assertTrue(n.image_queue.empty())

    def test_worker_processing_error_does_not_refresh_camera_health(self):
        n = self.node()
        n.tracker = Mock()
        n.image_queue.put((object(), 10.0, 1.0))
        def fail(_):
            n.worker_stop.set()
            raise ValueError('bad image')
        with patch.object(visual_adapter.time, 'monotonic', return_value=10.1), patch.object(visual_adapter, 'image_to_bgr', side_effect=fail):
            n.vision_worker()
        self.assertNotIn('camera', n.receipts)
        self.assertEqual(n.journal.append.call_args.args[0]['kind'], 'vision_error')
        n.tracker.update.assert_not_called()
