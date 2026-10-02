"""Adapter unit tests with ROS message/service doubles, not a ROS runtime test."""
from collections import deque
from dataclasses import asdict
import importlib
import math
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import cv2  # Import native extensions before patch.dict restores sys.modules.
import numpy
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight'))
from we_meet_flight.config import Config
from we_meet_flight.mission import Command, Mission, Sensors


class TwistDouble:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.twist = NS(linear=NS(x=0, y=0, z=0), angular=NS(x=0, y=0, z=0))


def load_adapter():
    names = ['rclpy', 'rclpy.node', 'rclpy.qos', 'ament_index_python', 'ament_index_python.packages',
             'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
             'mavros_msgs', 'mavros_msgs.msg', 'mavros_msgs.srv', 'rcl_interfaces',
             'rcl_interfaces.srv', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv']
    modules = {name: ModuleType(name) for name in names}
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].qos_profile_sensor_data = object()
    modules['ament_index_python.packages'].get_package_share_directory = lambda _: '/unused'
    for name, symbols in [('geometry_msgs.msg', ['PoseStamped', 'TwistStamped']),
                          ('sensor_msgs.msg', ['BatteryState', 'Image', 'Imu', 'Range']),
                          ('mavros_msgs.msg', ['EstimatorStatus', 'ExtendedState', 'State']),
                          ('mavros_msgs.srv', ['CommandBool', 'SetMode']),
                          ('rcl_interfaces.srv', ['GetParameters']), ('std_msgs.msg', ['String']),
                          ('std_srvs.srv', ['Trigger'])]:
        for symbol in symbols:
            setattr(modules[name], symbol, type(symbol, (), {'Request': NS}))
    modules['geometry_msgs.msg'].TwistStamped = TwistDouble
    modules['std_msgs.msg'].String = lambda **kwargs: NS(**kwargs)
    with patch.dict(sys.modules, modules):
        return importlib.import_module('we_meet_flight.node')


adapter = load_adapter()


class AdapterTests(unittest.TestCase):
    def node(self):
        n = adapter.FlightNode.__new__(adapter.FlightNode)
        n.c, n.s = Config(), Sensors()
        n.receipts, n.source_stamps = {}, {}
        n.journal = NS(append=Mock(), error='')
        return n


    def test_raw_lidar_projection_offset_and_capability(self):
        n = self.node()
        from dataclasses import replace
        n.c = replace(n.c, lidar_input_is_vertical_height=False, lidar_body_down_offset_m=0.1)
        n.roll, n.pitch = 0.1, 0.2
        n.range_samples = deque()
        n.telemetry = {}
        n.receipt = lambda *args: 1.0
        n.on_range(NS(range=3.0, min_range=0.1, max_range=8.0))
        self.assertAlmostEqual(n.s.range_m, 3.1*math.cos(0.1)*math.cos(0.2))
        n.on_range(NS(range=1.0, min_range=0.1, max_range=2.0))
        self.assertFalse(n.lidar_capable)
        self.assertTrue(math.isnan(n.s.range_m))

    def test_corrected_lidar_not_projected_twice(self):
        n = self.node()
        n.roll, n.pitch = 0.1, 0.2
        n.range_samples, n.telemetry = deque(), {}
        n.receipt = lambda *args: 1.0
        n.on_range(NS(range=3.0, min_range=0.1, max_range=8.0))
        self.assertEqual(n.s.range_m, 3.0)

    def test_dry_run_never_calls_fc_services(self):
        n = self.node()
        n.dry = True
        n.mode_client, n.arm_client = Mock(), Mock()
        for action in ['OFFBOARD', 'ARM', 'LAND']:
            n.request(action, 0)
        n.mode_client.call_async.assert_not_called()
        n.arm_client.call_async.assert_not_called()

    def test_live_start_requires_verified_configuration_and_camera(self):
        n = self.node()
        n.dry = False
        n.settings_verified = False
        n.axes_verified = False
        n.lidar_verified = False
        n.lidar_capable = False
        n.mission = Mission(n.c)
        response = n.on_start(None, NS())
        self.assertFalse(response.success)
        self.assertEqual(n.mission.state, 'IDLE')
        n.settings_verified = True
        response = n.on_start(None, NS())
        self.assertFalse(response.success)
        self.assertEqual(n.mission.state, 'IDLE')

    def test_source_age_duplicate_and_future_frame_rejected(self):
        n = self.node()
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
        msg = NS(header=NS(stamp=NS(sec=9, nanosec=900_000_000)))
        self.assertIsNotNone(n.receipt('camera', msg))
        self.assertIsNone(n.receipt('camera', msg))
        msg.header.stamp.sec = 11
        self.assertIsNone(n.receipt('camera', msg))
        msg.header.stamp.sec, msg.header.stamp.nanosec = 8, 0
        self.assertIsNone(n.receipt('camera', msg))

    def test_nonfinite_imu_is_not_fresh(self):
        n = self.node()
        n.receipt = lambda *args: 1
        n.receipts['imu'] = 1
        n.on_imu(NS(angular_velocity=NS(x=0, y=math.nan, z=0)))
        self.assertNotIn('imu', n.receipts)

    def test_land_service_failure_does_not_restart_offboard(self):
        n = self.node()
        n.mission = Mission(n.c)
        n.mission.state = 'LANDING'
        n.service_failure('LAND', 2, 'rejected')
        self.assertEqual(n.mission.state, 'FAILED')
        self.assertFalse(n.mission.step(3, n.s).publish)

    def test_single_message_contains_linear_and_yaw(self):
        n = self.node()
        n.dry = True
        n.last_tick_s = time.monotonic()
        n.snapshot = lambda _: n.s
        n.check_graph_and_frame = lambda _: None
        n.pending_services = []
        n.publisher = Mock()
        n.status_pub = Mock()
        n.output_topic = '/test_flying_v2/debug/cmd_vel'
        n.telemetry = {}
        n.frame_verified = False
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000, to_msg=lambda: NS(sec=1, nanosec=0)))
        n.mission = NS(state='CRUISE', result='pending', reason='', events=[], yaw_ref=0,
                       commanded_distance_m=1.0, hold_since=None, integrate_previous=True,
                       step=lambda *_: Command(0.1, -0.05, 0.02, 0.03, True),
                       acknowledge_published=Mock())
        n.tick()
        n.publisher.publish.assert_called_once()
        message = n.publisher.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual((message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
                          message.twist.angular.z), (0.1, -0.05, 0.02, 0.03))
