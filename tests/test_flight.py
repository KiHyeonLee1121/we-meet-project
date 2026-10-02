from dataclasses import asdict, fields, replace
import math
from pathlib import Path
import sys
import tempfile
import unittest
for package in ['we_meet_flight', 'we_meet_flight_core']:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src'/package))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import cv2
import numpy as np
from we_meet_flight_core.config import Config as CoreConfig
from we_meet_flight_core.mission import Mission as CoreMission, Sensors as CoreSensors
from we_meet_flight.config import Config, load_config
from we_meet_flight.image_io import image_to_bgr
from we_meet_flight.mission import Mission, Sensors
from we_meet_flight.vision import Detection, Observation, PanelDetector, TargetTracker
from offline_demo import run_demo
from test_mission import healthy, follow_command


class Harness:
    def __init__(self, vision=True):
        self.c = Config(vision_enabled=vision)
        self.m, self.s = Mission(self.c), Sensors(**asdict(healthy()), camera_age=0)
        self.now = 0.0
        self.m.start(0, self.s, 0)

    def tick(self, dt=0.05, visible=None, ex=0, ey=0, publish=True):
        self.now += dt
        follow_command(self.s, self.m, self.now)
        if visible is not None:
            self.s.observation = Observation(self.now, round(self.now*100), visible, ex, ey, 0.1, 0.9, 1)
        cmd = self.m.step(self.now, self.s)
        if publish and cmd.publish:
            self.m.record_publish(cmd, self.now)
        if cmd.request == 'OFFBOARD':
            self.s.mode = 'OFFBOARD'
        elif cmd.request == 'ARM':
            self.s.armed, self.s.landed = True, False
        return cmd

    def to_state(self, state, **kwargs):
        for _ in range(3000):
            if self.m.state == state:
                return
            self.tick(**kwargs)
        raise AssertionError((state, self.m.state, self.m.reason))


class CommonLogicParityTests(unittest.TestCase):
    def test_all_shared_defaults_and_profiles_match(self):
        base, visual = CoreConfig(), Config()
        for field in fields(CoreConfig):
            self.assertEqual(getattr(base, field.name), getattr(visual, field.name), field.name)
        directory = Path(__file__).resolve().parents[1]/'ros2_ws/src'
        import we_meet_flight_core.config as core_config
        reference = core_config.load_config(directory/'we_meet_flight_core/config/velocity_trial.yaml')
        for profile in ['baseline.yaml', 'panel_approach.yaml']:
            loaded = load_config(directory/'we_meet_flight/config'/profile)
            for field in fields(CoreConfig):
                self.assertEqual(getattr(reference, field.name), getattr(loaded, field.name), (profile, field.name))

    def compare_commands(self, vision):
        # Independent controller instances see identical sensor history and dispatch times.
        core = CoreMission(CoreConfig())
        s = healthy()
        visual = Harness(vision=vision)
        core.start(0, s, 0)
        periods = [0.03, 0.08, 0.05, 0.10, 0.04]
        states = set()
        for i in range(2000):
            s.yaw = visual.s.yaw = 0.015*math.sin(i/11)
            s.height_m = visual.s.height_m = 2.8 if i < 80 else 3.0
            a = visual.tick(dt=periods[i % len(periods)], visible=False)
            follow_command(s, core, visual.now)
            b = core.step(visual.now, s)
            # Camera-enabled no-target flight intentionally fails after common BRAKE.
            if vision and core.state == 'ZERO_VELOCITY_HOLD':
                self.assertEqual(visual.m.state, 'LANDING')
                break
            self.assertEqual(a, b, (i, visual.m.state, core.state))
            self.assertEqual(visual.m.state, core.state)
            if b.publish:
                core.record_publish(b, visual.now)
            self.assertEqual(visual.m.commanded_distance_m, core.commanded_distance_m)
            states.add(core.state)
            if b.request == 'OFFBOARD':
                s.mode = 'OFFBOARD'
            elif b.request == 'ARM':
                s.armed, s.landed = True, False
            elif b.request == 'LAND':
                s.mode = visual.s.mode = 'AUTO.LAND'
            if core.state == 'LANDING' and not vision:
                s.landed = visual.s.landed = True
                s.armed = visual.s.armed = False
            if core.state == 'COMPLETE':
                break
        self.assertTrue({'PRESTREAM', 'OFFBOARD_WAIT', 'ARM_WAIT', 'ASCEND', 'SETTLE', 'ADVANCE', 'BRAKE'} <= states)
        self.assertEqual(core.yaw_ref, visual.m.yaw_ref)

    def test_camera_disabled_entire_trial_matches_core(self):
        self.compare_commands(False)

    def test_camera_enabled_no_panel_matches_until_end_of_braking(self):
        self.compare_commands(True)

    def test_dispatch_latency_integrals_match(self):
        a, b = CoreMission(CoreConfig()), Mission(Config(vision_enabled=False))
        sa, sb = healthy(), Sensors(**asdict(healthy()))
        a.start(0, sa, 0); b.start(0, sb, 0)
        now = 0
        for i in range(1200):
            now += [0.03, 0.08, 0.05, 0.10, 0.04][i % 5]
            follow_command(sa, a, now)
            follow_command(sb, b, now)
            ca, cb = a.step(now, sa), b.step(now, sb)
            self.assertEqual(ca, cb)
            if ca.publish:
                dispatch = now+[0.001, 0.006, 0.012][i % 3]
                self.assertEqual(a.record_publish(ca, dispatch), b.record_publish(cb, dispatch))
            for sensor in [sa, sb]:
                if ca.request == 'OFFBOARD': sensor.mode = 'OFFBOARD'
                if ca.request == 'ARM': sensor.armed, sensor.landed = True, False
                if ca.request == 'LAND': sensor.mode = 'AUTO.LAND'
            if a.state == 'LANDING': break
        self.assertEqual(a.result, 'expected_5m_zero_command_5s')
        self.assertEqual(a.commanded_distance_m, b.commanded_distance_m)


class CameraFlightTests(unittest.TestCase):
    def test_panel_acquisition_smoothly_brakes_then_aligns(self):
        h = Harness(); h.to_state('ADVANCE')
        for _ in range(50): h.tick()
        before = h.m.xy
        cmd = h.tick(visible=True, ey=-0.4)
        self.assertEqual(h.m.state, 'BRAKE')
        self.assertGreater(cmd.vx, 0)
        self.assertLess(cmd.vx, before[0])
        while h.m.state == 'BRAKE':
            before = h.m.xy
            cmd = h.tick(visible=True, ey=-0.4)
            self.assertLessEqual(math.hypot(cmd.vx-before[0], cmd.vy-before[1]), 0.01250001)
        self.assertEqual(h.m.state, 'ALIGN')
        for _ in range(20): cmd = h.tick(visible=True, ey=-0.4)
        self.assertGreater(cmd.vx, 0)
        self.assertLessEqual(math.hypot(cmd.vx, cmd.vy), 0.12)

    def test_alignment_uses_current_body_yaw_but_reference_stays_fixed(self):
        h = Harness(); h.to_state('ADVANCE')
        h.to_state('ALIGN', visible=True, ey=-0.3)
        h.s.yaw = 0.05
        for _ in range(20): cmd = h.tick(visible=True, ey=-0.3)
        self.assertAlmostEqual(cmd.vy/cmd.vx, math.tan(0.05))
        self.assertEqual(h.m.yaw_ref, 0)

    def test_hold_requires_continuous_fresh_center_height_and_heading(self):
        for field, value in [('center', 0.25), ('height_m', 3.2), ('yaw', 0.15), ('visible', False)]:
            with self.subTest(field=field):
                h = Harness(); h.to_state('ADVANCE'); h.to_state('VISUAL_HOLD', visible=True)
                for _ in range(50): h.tick(visible=True)
                if field == 'center': h.tick(visible=True, ex=value)
                elif field == 'visible': h.tick(visible=False)
                else:
                    setattr(h.s, field, value); h.tick(visible=True); setattr(h.s, field, 3 if field == 'height_m' else 0)
                self.assertIsNone(h.m.hold_since)
                for _ in range(90): h.tick(visible=True)
                self.assertNotEqual(h.m.state, 'LANDING')
                h.to_state('LANDING', visible=True)
                self.assertEqual(h.m.result, 'panel_centered_5s')

    def test_hold_starts_at_actual_zero_publication(self):
        h = Harness(); h.to_state('ADVANCE'); h.to_state('ALIGN', visible=True, ey=-0.4)
        for _ in range(30): h.tick(visible=True, ey=-0.4)
        # Decelerate with successful publications until the last non-zero step.
        while math.hypot(*h.m.xy) > h.c.horizontal_accel_mps2*0.05+1e-12:
            h.tick(visible=True)
        h.tick(visible=True, publish=False)
        self.assertIsNone(h.m.hold_since)
        self.assertEqual(h.m.xy, (0.0, 0.0))
        self.assertIsNone(h.m.hold_since)
        # Manually record zero publication with real dispatch latency.
        cmd = h.tick(visible=True, publish=False)
        h.m.record_publish(cmd, h.now+0.01)
        self.assertAlmostEqual(h.m.hold_since, h.now+0.01)

    def test_visual_hold_restarts_when_estimated_motion_exceeds_limit(self):
        h = Harness(); h.to_state('ADVANCE'); h.to_state('VISUAL_HOLD', visible=True)
        for _ in range(40): h.tick(visible=True)
        first = h.m.hold_since
        h.now += .05; h.s.odometry_stamp_s = h.now
        h.s.velocity_enu = (.25, 0, 0)
        h.s.observation = Observation(h.now, round(h.now*100), True, 0, 0, .1, .9, 1)
        cmd = h.m.step(h.now, h.s); h.m.record_publish(cmd, h.now)
        self.assertIsNone(h.m.hold_since)
        self.assertEqual((cmd.vx, cmd.vy), (0, 0))
        h.tick(visible=True)
        self.assertGreater(h.m.hold_since, first)
        for _ in range(80): h.tick(visible=True)
        self.assertEqual(h.m.state, 'VISUAL_HOLD')

    def test_locked_panel_loss_during_braking_never_resumes_forward(self):
        h = Harness(); h.to_state('ADVANCE')
        for _ in range(30): h.tick()
        h.tick(visible=True, ey=-0.3)
        last = h.m.speed
        for _ in range(20):
            cmd = h.tick(visible=False)
            self.assertLessEqual(math.hypot(cmd.vx, cmd.vy), last+1e-12)
            last = math.hypot(cmd.vx, cmd.vy)
            if h.m.state == 'LANDING': break
        self.assertEqual(h.m.result, 'aborted')
        self.assertIn('lost', h.m.reason)

    def test_target_loss_alignment_and_stale_camera_abort(self):
        for fault in ['loss', 'camera', 'estimator', 'critical']:
            h = Harness(); h.to_state('ADVANCE'); h.to_state('ALIGN', visible=True, ey=-0.3)
            if fault == 'camera': h.s.camera_age = 1
            if fault == 'estimator': h.s.estimator_valid = False
            if fault == 'critical': h.s.failsafe_observed = True
            h.to_state('LANDING' if fault != 'critical' else 'RELEASED', visible=False)
            self.assertEqual(h.m.result, 'aborted')
            if fault == 'estimator': self.assertFalse(h.m.landing_stream_allowed)

    def test_no_panel_at_distance_budget_is_failure(self):
        h = Harness(); h.to_state('LANDING', visible=False)
        self.assertEqual(h.m.result, 'aborted')
        self.assertLess(abs(h.m.commanded_distance_m-5), 0.15)

    def test_visual_phase_timeout_uses_common_abort(self):
        h = Harness(); h.to_state('ADVANCE'); h.to_state('ALIGN', visible=True, ey=-0.4)
        h.m.entered_s = h.now-h.c.align_timeout_s
        self.assertEqual(h.tick(visible=True, ey=-0.4).request, 'LAND')
        self.assertIn('ALIGN phase timeout', h.m.reason)

    def test_opencv_closed_loop_and_baseline_end_to_end(self):
        for vision, expected in [(True, 'panel_centered_5s'), (False, 'expected_5m_zero_command_5s')]:
            result = run_demo(None, vision=vision)
            self.assertEqual(result['state'], 'COMPLETE')
            self.assertEqual(result['result'], expected)


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
        for kwargs in [{'zero_velocity_hold_s': math.nan}, {'horizontal_accel_mps2': 0},
                       {'image_to_body': [[1, 0], [1, 0]]}, {'vision_enabled': 'false'},
                       {'min_panel_area_fraction': 0.9}, {'acquisition_frames': 2.5},
                       {'visual_hold_timeout_s': 4}, {'vision_enabled': True, 'advance_enabled': False}]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError): Config(**kwargs)

    def test_legacy_yaml_preserves_values_and_rejects_conflicts(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'old.yaml'
            path.write_text('target_range_m: 2.8\napproach_distance_m: 4.9\ncruise_speed_mps: 0.3\nhover_s: 6.0\nlidar_input_is_vertical_height: true\n')
            config = load_config(path)
            self.assertEqual((config.target_height_m, config.commanded_target_distance_m, config.forward_speed_mps, config.zero_velocity_hold_s), (2.8, 4.9, 0.3, 6.0))
            self.assertTrue(config.lidar_input_is_vertical_height)
            path.write_text('target_range_m: 2.8\ntarget_height_m: 3.0\n')
            with self.assertRaises(ValueError): load_config(path)
