from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight_core'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from we_meet_flight_core.config import Config, load_config
from we_meet_flight_core.control import body_to_enu, quaternion_yaw, yaw_rate_command, yaw_reset_innovation
from we_meet_flight_core.journal import Journal
from we_meet_flight_core.mission import Command, Mission, Sensors
from core_offline_demo import run_demo
from replay_commands import replay


def healthy():
    return Sensors(connected=True, mode='POSCTL', landed=True, state_age=0, landed_age=0,
                   estimator_valid=True, estimator_age=0, yaw=0, yaw_age=0, imu_age=0,
                   height_m=3, height_rate_mps=0, range_age=0, battery_remaining=0.8, battery_age=0,
                   odometry_age=0, odometry_stamp_s=0, fc_reset_counter=2,
                   position_enu=(0, 0, 0), velocity_enu=(0, 0, 0), position_sigma_m=0.1,
                   velocity_sigma_mps=0.05, gps_age=0, gps_sigma_m=0.2, yaw_rate_rps=0)


def follow_command(s, m, now):
    """Ideal synthetic estimator for legacy controller tests, never used live."""
    old = m.last_published_command or Command()
    dt = now-m.last_publish if m.last_publish is not None else 0
    s.position_enu = (m.launch_xy[0]+m.expected_xy[0]+old.vx*dt,
                      m.launch_xy[1]+m.expected_xy[1]+old.vy*dt, 0)
    s.velocity_enu = (old.vx, old.vy, s.height_rate_mps)
    s.odometry_stamp_s = now


class Harness:
    def __init__(self, c=None):
        self.m, self.s = Mission(c or Config()), healthy()
        self.now = 0.0
        self.m.start(self.now, self.s, 0)
        self.commands = []

    def tick(self, dt=0.05, publish=True, automatic=True):
        self.now += dt
        follow_command(self.s, self.m, self.now)
        cmd = self.m.step(self.now, self.s)
        if cmd.publish and publish:
            self.m.record_publish(cmd, self.now)
        if automatic:
            if cmd.request == 'OFFBOARD':
                self.s.mode = 'OFFBOARD'
            elif cmd.request == 'ARM':
                self.s.armed = True
                self.s.landed = False
        self.commands.append(cmd)
        return cmd

    def to_state(self, state):
        for _ in range(3000):
            if self.m.state == state:
                return
            self.tick()
        raise AssertionError((state, self.m.state, self.m.reason))


class MissionTests(unittest.TestCase):
    def test_complete_ascent_advance_brake_zero_hold_land(self):
        h = Harness()
        h.to_state('ZERO_VELOCITY_HOLD')
        self.assertLess(abs(h.m.commanded_distance_m-5), 0.15)
        first_zero = h.m.hold_since
        h.to_state('LANDING')
        self.assertGreaterEqual(h.now-first_zero, 5)
        self.assertEqual(h.m.result, 'expected_5m_zero_command_5s')
        self.assertTrue(h.m.zero_hold_completed)
        h.s.mode = 'AUTO.LAND'
        self.assertFalse(h.tick().publish)
        h.s.armed, h.s.landed = False, True
        h.tick()
        self.assertEqual(h.m.state, 'COMPLETE')

    def test_ascent_only_never_generates_forward_speed(self):
        h = Harness(Config(advance_enabled=False))
        h.to_state('LANDING')
        self.assertEqual(h.m.commanded_distance_m, 0)
        self.assertTrue(all(c.vx == c.vy == 0 for c in h.commands))

    def test_actual_publish_timestamp_signed_projection(self):
        m = Mission(Config())
        m.yaw_ref = math.pi/2
        c = Command(0.1, -0.2, 2, 0, True, integrate_distance=True)
        m.record_publish(c, 3)
        seg = m.record_publish(Command(publish=True), 3.08)
        self.assertAlmostEqual(m.commanded_distance_m, -0.016)
        self.assertAlmostEqual(seg['forward_mps'], -0.2)
        self.assertAlmostEqual(seg['dt_s'], 0.08)

    def test_distance_prediction_does_not_duplicate_integration(self):
        m = Mission(Config()); m.yaw_ref = 0
        m.record_publish(Command(0.2, publish=True, integrate_distance=True), 1)
        for _ in range(10):
            self.assertAlmostEqual(m.distance_at(1.05), 0.01)
        self.assertEqual(m.commanded_distance_m, 0)
        m.record_publish(Command(publish=True), 1.05)
        self.assertAlmostEqual(m.commanded_distance_m, 0.01)

    def test_long_gap_not_integrated_or_declared_success(self):
        h = Harness(); h.to_state('ADVANCE')
        for _ in range(10): h.tick()
        before = h.m.commanded_distance_m
        c = h.tick(1, publish=False)
        self.assertEqual(h.m.state, 'LANDING')
        self.assertFalse(c.publish)
        self.assertEqual(c.request, 'LAND')
        self.assertEqual(h.m.commanded_distance_m, before)
        self.assertFalse(h.m.zero_hold_completed)

    def test_publish_gap_record_rejects_catchup(self):
        m = Mission(Config()); m.yaw_ref = 0
        m.record_publish(Command(0.2, publish=True, integrate_distance=True), 1)
        with self.assertRaises(ValueError): m.record_publish(Command(publish=True), 2)
        self.assertEqual(m.commanded_distance_m, 0)

    def test_no_prestream_publication_blocks_offboard_request(self):
        h = Harness()
        for _ in range(30):
            c = h.tick(publish=False)
            self.assertNotEqual(c.request, 'OFFBOARD')
        self.assertEqual(h.m.state, 'RELEASED')

    def test_offboard_and_arm_require_observed_state_not_request(self):
        h = Harness()
        for _ in range(30): h.tick(automatic=False)
        self.assertEqual(h.m.state, 'OFFBOARD_WAIT')
        self.assertTrue(all(c.vz == 0 for c in h.commands))
        for _ in range(110): h.tick(automatic=False)
        self.assertEqual(h.m.state, 'RELEASED')

    def test_fixed_reference_and_same_yaw_helper_every_stage(self):
        h = Harness(); h.s.yaw = 0.02
        h.to_state('LANDING')
        active = [c for c in h.commands if c.publish]
        self.assertTrue(active)
        self.assertTrue(all(abs(c.yaw_rate+0.02) < 1e-9 for c in active))
        self.assertTrue(all(abs(c.vy) < 1e-9 for c in active))
        self.assertEqual(h.m.yaw_ref, 0)

    def test_brake_starts_before_five_and_ramp_is_bounded(self):
        h = Harness(); h.to_state('ADVANCE')
        while h.m.state != 'BRAKE':
            before = h.m.speed
            h.tick()
            self.assertLessEqual(abs(h.m.speed-before), 0.25*0.05+1e-9)
        self.assertLess(h.m.commanded_distance_m, 5)
        self.assertGreater(h.m.speed, 0)
        before_distance = h.m.commanded_distance_m
        h.to_state('ZERO_VELOCITY_HOLD')
        self.assertGreater(h.m.commanded_distance_m, before_distance)

    def test_irregular_periods_end_with_command_tolerance(self):
        for seed in range(12):
            h = Harness(); random_source = random.Random(seed)
            for _ in range(1500):
                h.tick(dt=random_source.uniform(0.025, 0.15))
                if h.m.state == 'LANDING': break
            self.assertTrue(h.m.zero_hold_completed, (seed, h.m.state, h.m.reason))
            self.assertLess(abs(h.m.commanded_distance_m-5), 0.15)

    def test_invalid_sensor_estimator_yaw_aborts(self):
        for field, value in [('range_age', 1), ('height_m', math.nan), ('height_rate_mps', math.inf),
                             ('yaw_age', 1), ('yaw', math.nan), ('yaw_reset', True), ('imu_age', 1),
                             ('estimator_valid', False), ('estimator_age', 3), ('battery_remaining', 0.01)]:
            with self.subTest(field=field):
                h = Harness(); h.to_state('ADVANCE'); setattr(h.s, field, value)
                c = h.tick()
                self.assertEqual(h.m.state, 'LANDING')
                self.assertEqual(c.request, 'LAND')
                self.assertFalse(h.m.zero_hold_completed)
                if field.startswith('estimator') or field.startswith('yaw') or field == 'imu_age':
                    self.assertFalse(c.publish)

    def test_takeover_or_failsafe_releases_without_reentry(self):
        for field, value in [('mode', 'POSCTL'), ('mode', 'AUTO.LAND'), ('failsafe_observed', True), ('connected', False)]:
            h = Harness(); h.to_state('ADVANCE'); setattr(h.s, field, value)
            for _ in range(20):
                c = h.tick()
                self.assertFalse(c.publish)
                self.assertFalse(c.request)
            self.assertEqual(h.m.state, 'RELEASED')

    def test_phase_timeout_aborts(self):
        h = Harness(Config(ascend_timeout_s=0.5)); h.s.height_m = 0.3
        h.to_state('LANDING')
        self.assertIn('phase timeout', h.m.reason)

    def test_height_excursion_restarts_continuous_five_second_hold(self):
        h = Harness(); h.to_state('ZERO_VELOCITY_HOLD')
        start = h.m.hold_since
        self.assertIsNotNone(start)
        h.s.height_m = 3.2
        for _ in range(20): h.tick()
        self.assertIsNone(h.m.hold_since)
        h.s.height_m = 3
        h.tick()
        self.assertGreater(h.m.hold_since, start)
        for _ in range(80): h.tick()
        self.assertEqual(h.m.state, 'ZERO_VELOCITY_HOLD')
        h.to_state('LANDING')
        self.assertTrue(h.m.zero_hold_completed)

    def test_landing_refusal_timeout_no_airborne_disarm(self):
        h = Harness(Config(landing_timeout_s=0.5)); h.to_state('LANDING')
        for _ in range(20): h.tick()
        self.assertEqual(h.m.state, 'FAILED')
        self.assertTrue(h.s.armed)
        self.assertTrue(all(c.request in {'', 'OFFBOARD', 'ARM', 'LAND'} for c in h.commands))

    def test_estimator_loss_in_land_never_resumes_stream(self):
        h = Harness(); h.to_state('LANDING')
        h.s.estimator_valid = False
        self.assertFalse(h.tick().publish)
        h.s.estimator_valid = True
        self.assertFalse(h.tick().publish)

    def test_complete_offline_and_log_replay(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'trial.jsonl'
            result = run_demo(path, irregular=True)
            self.assertEqual(result['state'], 'COMPLETE')
            replayed = replay(path)
            self.assertEqual(replayed['invalid_intervals'], 0)
            self.assertAlmostEqual(replayed['recomputed_commanded_distance_m'], result['commanded_distance_m'])


class ConfigAndLogTests(unittest.TestCase):
    def test_configs_and_invalid_parameters(self):
        directory = Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight_core/config'
        self.assertTrue(load_config(directory/'velocity_trial.yaml').advance_enabled)
        self.assertFalse(load_config(directory/'ascent_only.yaml').advance_enabled)
        self.assertFalse(Config().lidar_input_is_vertical_height)
        for changes in [{'forward_speed_mps': 0}, {'yaw_kp': math.nan}, {'advance_enabled': 1},
                        {'lidar_body_down_offset_m': math.inf}, {'prestream_s': 0.1}, {'hold_timeout_s': 4}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError): Config(**changes)

    def test_yaw_wrap_saturation_quaternion_and_coordinate_signs(self):
        self.assertAlmostEqual(yaw_rate_command(target_rad=-math.pi+0.02, current_rad=math.pi-0.02,
                                               kp=1, maximum_rate_rad_s=0.35), 0.04)
        self.assertEqual(yaw_rate_command(target_rad=2, current_rad=0, kp=1, maximum_rate_rad_s=0.35), 0.35)
        self.assertAlmostEqual(body_to_enu(1, 0, math.pi/2)[1], 1)
        self.assertAlmostEqual(yaw_reset_innovation(0, 0.02, 0.2, 0.1), 0)
        self.assertGreater(yaw_reset_innovation(0, 0.12, 0, 0.1), 0.1)
        with self.assertRaises(ValueError): quaternion_yaw(0, 0, 0, 0)

    def test_automatic_files_missing_value_null_and_summary(self):
        with tempfile.TemporaryDirectory() as root:
            j = Journal(root, {'config': {'target_height_m': 3}})
            row = {'kind': 'control', 'time_s': 1, 'dt_s': 0.05, 'state': 'COMPLETE',
                   'result': 'expected_5m_zero_command_5s', 'reason': '',
                   'command': asdict(Command()), 'sensors': asdict(healthy()), 'telemetry': {},
                   'commanded_distance_m': 4.97, 'zero_hold_completed': True}
            # Missing XY values are log-only and must not break flight logging.
            row['telemetry']['position_enu_log_only'] = [math.nan, math.inf, 3]
            j.append(row); j.close()
            self.assertFalse(j.error)
            for name in ['run_metadata.json', 'config_snapshot.yaml', 'telemetry.csv', 'commands.jsonl',
                         'events.jsonl', 'summary.json', 'console.log', 'field_result.md']:
                self.assertTrue((j.directory/name).exists())
            summary = json.loads((j.directory/'summary.json').read_text())
            self.assertTrue(summary['landing_confirmed'])
            self.assertTrue(summary['zero_velocity_command_hold_completed'])
            self.assertIsNone(summary['estimated_forward_displacement_m_log_only'])

    def test_zero_sensor_defaults_do_not_bypass_start(self):
        m = Mission(Config())
        with self.assertRaises(ValueError): m.start(0, Sensors(), 0)
