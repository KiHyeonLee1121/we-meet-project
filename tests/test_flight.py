from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import cv2
import numpy as np
from we_meet_flight.config import Config, load_config
from we_meet_flight.control import body_to_enu, yaw_rate_command, yaw_reset_innovation
from we_meet_flight.image_io import image_to_bgr
from we_meet_flight.mission import Command, Mission, Sensors
from we_meet_flight.vision import Detection, Observation, PanelDetector, TargetTracker
from offline_demo import run_demo


def healthy(vision=False):
    return Sensors(estimator_valid=True, estimator_age=0, connected=True, armed=False, mode='POSCTL', landed=True,
                   state_age=0, landed_age=0, yaw=0, yaw_age=0, imu_age=0,
                   range_m=3, range_rate=0, range_age=0,
                   battery_remaining=0.8, battery_age=0, camera_age=0)


class Harness:
    def __init__(self, vision=False):
        self.c = Config(vision_enabled=vision)
        self.m, self.s = Mission(self.c), healthy(vision)
        self.now = 0
        self.m.start(0, self.s, 0)

    def tick(self, dt=0.05, visible=None, ex=0, ey=0, publish=True):
        self.now += dt
        if visible is not None:
            self.s.observation = Observation(self.now, round(self.now*100), visible, ex, ey, 0.1, 0.9, 1)
        cmd = self.m.step(self.now, self.s)
        self.m.acknowledge_published(publish and cmd.publish)
        if cmd.request == 'OFFBOARD':
            self.s.mode = 'OFFBOARD'
        elif cmd.request == 'ARM':
            self.s.armed = True
            self.s.landed = False
        return cmd

    def to_state(self, state, **kwargs):
        for _ in range(2000):
            if self.m.state == state:
                return
            self.tick(**kwargs)
        raise AssertionError(f'did not reach {state}: {self.m.state}, {self.m.reason}')


class FlightTests(unittest.TestCase):

    def test_estimator_loss_does_not_stream_zero_as_safe_hold(self):
        h = Harness()
        h.to_state('CRUISE')
        h.s.estimator_valid = False
        cmd = h.tick()
        self.assertEqual(cmd.request, 'LAND')
        self.assertFalse(cmd.publish)
        self.assertEqual(h.m.result, 'aborted')

    def test_ascent_only_has_no_forward_command(self):
        h = Harness()
        h.m.c = replace(h.c, approach_enabled=False)
        h.to_state('HOLD')
        self.assertEqual(h.m.commanded_distance_m, 0)
        self.assertEqual(h.m.xy, (0, 0))
        h.to_state('LANDING')

    def test_signed_integration_and_irregular_clock(self):
        h = Harness()
        h.to_state('CRUISE')
        total = 0
        for dt in [0.03, 0.08, 0.04, 0.12, 0.05]*20:
            previous = h.m.last_command
            count = h.m.integrate_previous and h.m.last_published
            h.tick(dt=dt)
            if count:
                total += previous.vx*dt
        self.assertAlmostEqual(h.m.commanded_distance_m, total)
        h.m.last_command = Command(-0.1, 0, 0, 0, True)
        h.m.last_published = h.m.integrate_previous = True
        before = h.m.commanded_distance_m
        h.tick()
        self.assertAlmostEqual(h.m.commanded_distance_m, before-0.005)

    def test_per_run_logs_and_replay(self):
        import json
        from we_meet_flight.journal import Journal
        from replay_commands import replay
        with tempfile.TemporaryDirectory() as root:
            journal = Journal(root, {'config': {'hover_s': 5.0}})
            for i in range(3):
                journal.append({'kind': 'control', 'time_s': 10+i*0.05, 'dt_s': 0.05,
                    'state': 'CRUISE', 'result': 'pending', 'reason': '',
                    'command': {'vx': 0.2, 'vy': 0, 'vz': 0, 'yaw_rate': 0},
                    'sensors': {'mode': 'OFFBOARD', 'armed': True, 'range_m': 3, 'yaw': 0},
                    'telemetry': {}, 'commanded_distance_m': i*0.01, 'published': True,
                    'yaw_ref': 0, 'integrate_distance': True})
            journal.close()
            self.assertFalse(journal.error)
            for name in ['run_metadata.json', 'config_snapshot.yaml', 'telemetry.csv',
                         'commands.jsonl', 'events.jsonl', 'summary.json', 'console.log', 'field_result.md']:
                self.assertTrue((journal.directory/name).exists())
            result = replay(journal.directory/'commands.jsonl')
            self.assertAlmostEqual(result['recomputed_commanded_distance_m'], 0.02)
            self.assertEqual(json.loads((journal.directory/'summary.json').read_text())['state'], 'CRUISE')

    def test_full_baseline_command_budget_and_hover(self):
        h = Harness()
        h.to_state('HOLD')
        self.assertLess(abs(h.m.commanded_distance_m-5), 0.025)
        self.assertIsNone(h.m.hold_since)
        h.tick()
        start = h.m.hold_since
        h.to_state('LANDING')
        self.assertGreaterEqual(h.now-start, 5)
        self.assertEqual(h.m.result, 'command_distance_hover_5s')

    def test_no_distance_for_unpublished_commands(self):
        h = Harness()
        h.to_state('CRUISE')
        for _ in range(10):
            h.tick(publish=False)
        self.assertEqual(h.m.commanded_distance_m, 0)

    def test_long_timer_gap_aborts_without_integrating_gap(self):
        h = Harness()
        h.to_state('CRUISE')
        for _ in range(20):
            h.tick()
        before = h.m.commanded_distance_m
        cmd = h.tick(dt=1)
        self.assertEqual(h.m.state, 'LANDING')
        self.assertEqual(cmd.request, 'LAND')
        self.assertEqual(h.m.commanded_distance_m, before)
        self.assertEqual((cmd.vx, cmd.vy, cmd.vz), (0, 0, 0))

    def test_yaw_hold_in_every_offboard_stage_and_fixed_reference(self):
        h = Harness()
        h.s.yaw = 0.02
        for _ in range(80):
            cmd = h.tick()
            self.assertAlmostEqual(cmd.yaw_rate, -0.02)
            self.assertEqual(h.m.yaw_ref, 0)
        h.to_state('CRUISE')
        cmd = h.tick()
        self.assertAlmostEqual(cmd.vy, 0)

    def test_takeover_never_reenters_or_requests_land(self):
        h = Harness()
        h.to_state('CRUISE')
        h.s.mode = 'POSCTL'
        cmd = h.tick()
        self.assertEqual(h.m.state, 'RELEASED')
        self.assertFalse(cmd.publish)
        for _ in range(100):
            cmd = h.tick()
            self.assertFalse(cmd.publish)
            self.assertFalse(cmd.request)

    def test_external_land_stops_publishing(self):
        h = Harness()
        h.to_state('HOLD')
        h.to_state('LANDING')
        h.s.mode = 'AUTO.LAND'
        self.assertFalse(h.tick().publish)
        h.s.landed, h.s.armed = True, False
        h.tick()
        self.assertEqual(h.m.state, 'DONE')

    def test_stale_yaw_lidar_camera_and_reset_abort(self):
        for field, value in [('yaw_age', 1), ('range_age', 1), ('camera_age', 1),
                             ('yaw_reset', True), ('yaw', math.nan), ('range_m', math.inf),
                             ('battery_remaining', 0.01)]:
            with self.subTest(field=field):
                h = Harness(vision=True)
                h.to_state('CRUISE')
                setattr(h.s, field, value)
                cmd = h.tick()
                self.assertEqual(h.m.state, 'LANDING')
                self.assertEqual(cmd.vx, 0)

    def test_panel_acquisition_smoothly_brakes_then_aligns(self):
        h = Harness(vision=True)
        h.to_state('CRUISE')
        for _ in range(50):
            before = h.m.xy
            cmd = h.tick()
            self.assertLessEqual(math.hypot(cmd.vx-before[0], cmd.vy-before[1]), 0.01250001)
        before = h.m.xy
        cmd = h.tick(visible=True, ey=-0.3)
        self.assertEqual(h.m.state, 'BRAKE')
        self.assertLess(cmd.vx, before[0])
        self.assertGreater(cmd.vx, 0)
        h.to_state('ALIGN', visible=True, ey=-0.3)
        for _ in range(5):
            cmd = h.tick(visible=True, ey=-0.3)
        self.assertGreater(cmd.vx, 0)
        self.assertLessEqual(math.hypot(cmd.vx, cmd.vy), 0.12)

    def test_vision_uses_current_body_yaw(self):
        h = Harness(vision=True)
        h.to_state('CRUISE')
        h.to_state('ALIGN', visible=True, ey=-0.3)
        h.s.yaw = 0.05
        for _ in range(20):
            cmd = h.tick(visible=True, ey=-0.3)
        self.assertAlmostEqual(cmd.vy/cmd.vx, math.tan(0.05))
        self.assertEqual(h.m.yaw_ref, 0)

    def test_hover_requires_continuous_fresh_centered_target(self):
        h = Harness(vision=True)
        h.to_state('CRUISE')
        h.to_state('HOLD', visible=True)
        for _ in range(50):
            h.tick(visible=True)
        h.tick(visible=True, ex=0.25)
        self.assertEqual(h.m.state, 'ALIGN')
        self.assertIsNone(h.m.hold_since)
        h.to_state('HOLD', visible=True)
        for _ in range(90):
            h.tick(visible=True)
        self.assertNotEqual(h.m.state, 'LANDING')
        h.to_state('LANDING', visible=True)
        self.assertEqual(h.m.result, 'panel_centered_5s')

    def test_target_loss_stops_and_aborts_without_forward_resume(self):
        h = Harness(vision=True)
        h.to_state('CRUISE')
        h.to_state('ALIGN', visible=True, ey=-0.3)
        for _ in range(10):
            h.tick(visible=True, ey=-0.3)
        h.tick(visible=False)
        h.to_state('LANDING', visible=False)
        self.assertEqual(h.m.result, 'aborted')
        self.assertIn('lost', h.m.reason)
        self.assertEqual(h.m.xy, (0, 0))

    def test_no_panel_at_budget_is_failure(self):
        h = Harness(vision=True)
        h.to_state('LANDING')
        self.assertEqual(h.m.result, 'aborted')
        self.assertLess(abs(h.m.commanded_distance_m-5), 0.025)

    def test_synthetic_camera_closed_loop_end_to_end(self):
        result = run_demo(None)
        self.assertEqual(result['state'], 'DONE')
        self.assertEqual(result['result'], 'panel_centered_5s')
        self.assertIn('BRAKE->ALIGN', result['transitions'])


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.c = Config()
        self.detector = PanelDetector(self.c)

    def test_detect_perspective_dark_panel_not_triangle_or_clipped(self):
        image = np.full((480, 640, 3), 220, dtype=np.uint8)
        cv2.fillConvexPoly(image, np.array([[200, 100], [390, 130], [360, 330], [180, 320]]), (35, 50, 70))
        ds = self.detector.detect(image)
        self.assertEqual(len(ds), 1)
        self.assertLess(abs(ds[0].cx-0.45), 0.06)
        blank = np.full_like(image, 220)
        self.assertEqual(self.detector.detect(blank), [])
        cv2.fillConvexPoly(blank, np.array([[100, 100], [300, 100], [200, 350]]), (20, 20, 20))
        self.assertEqual(self.detector.detect(blank), [])
        clipped = np.full_like(image, 220)
        cv2.rectangle(clipped, (200, 0), (400, 200), (20, 20, 20), -1)
        self.assertEqual(self.detector.detect(clipped), [])

    def test_acquire_distinct_frames_and_do_not_switch_targets(self):
        tracker = TargetTracker(self.c)
        d = Detection(0.3, 0.4, 0.1, 0.9, ())
        self.assertFalse(tracker.update([d], 0, 1).visible)
        self.assertIsNone(tracker.update([d], 0.1, 1))
        self.assertFalse(tracker.update([d], 0.2, 2).visible)
        self.assertTrue(tracker.update([d], 0.3, 3).visible)
        other = Detection(0.8, 0.8, 0.2, 1, ())
        self.assertFalse(tracker.update([other], 0.4, 4).visible)
        self.assertTrue(tracker.update([d, other], 0.5, 5).visible)
        self.assertFalse(tracker.update([other], 1.5, 6).visible)

    def test_padded_rgb_image_and_bad_length(self):
        from types import SimpleNamespace
        msg = SimpleNamespace(height=2, width=1, step=4, encoding='rgb8',
                              data=bytes([255, 0, 0, 1, 0, 255, 0, 2]))
        bgr = image_to_bgr(msg)
        self.assertEqual(bgr[0, 0].tolist(), [0, 0, 255])
        msg.data = b''
        with self.assertRaises(ValueError):
            image_to_bgr(msg)


class ConfigurationTests(unittest.TestCase):
    def test_invalid_configs_rejected(self):
        for kwargs in [{'hover_s': math.nan}, {'horizontal_accel_mps2': 0},
                       {'image_to_body': [[1, 0], [1, 0]]}, {'vision_enabled': 'false'},
                       {'min_panel_area_fraction': 0.9}, {'acquisition_frames': 2.5}]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Config(**kwargs)
        root = Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight/config'
        self.assertFalse(load_config(root/'baseline.yaml').vision_enabled)
        self.assertTrue(load_config(root/'panel_approach.yaml').vision_enabled)

    def test_yaw_wrap_and_physical_motion_vs_reset(self):
        self.assertAlmostEqual(yaw_rate_command(target_rad=-math.pi+0.02, current_rad=math.pi-0.02,
                                               kp=1, maximum_rate_rad_s=0.35), 0.04)
        self.assertEqual(yaw_rate_command(target_rad=2, current_rad=0, kp=1, maximum_rate_rad_s=0.35), 0.35)
        self.assertAlmostEqual(yaw_reset_innovation(0, 0.02, 0.2, 0.1), 0)
        self.assertGreater(yaw_reset_innovation(0, 0.12, 0, 0.1), 0.1)
