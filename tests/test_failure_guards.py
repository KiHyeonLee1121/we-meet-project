"""Failure-driven acceptance tests. No hardware writes or ROS runtime."""
from dataclasses import replace
import math
from pathlib import Path
import struct
from types import SimpleNamespace as NS
import unittest

from test_adapter import adapter
from test_mission import Harness, healthy
from we_meet_flight_core.config import Config
from we_meet_flight_core.mission import Mission
from we_meet_flight_core.telemetry import decode_odometry, horizontal_sigma


def packet(counter=2, timestamp=1000000, length=232):
    payload = bytearray(233)
    struct.pack_into('<Q', payload, 0, timestamp)
    struct.pack_into('<3f', payload, 8, 2, 3, -4)
    struct.pack_into('<3f', payload, 36, .1, .2, -.3)
    pc, vc = [math.nan]*21, [math.nan]*21
    for i in (0, 6, 11): pc[i], vc[i] = .01, .0025
    struct.pack_into('<21f', payload, 60, *pc)
    struct.pack_into('<21f', payload, 144, *vc)
    payload[228:232] = bytes([1, 1, counter, 8])
    payload.extend(b'\0'*7)
    return NS(msgid=331, framing_status=1, magic=253, sysid=1, compid=1, len=length,
              payload64=list(struct.unpack('<30Q', payload)))


class TelemetryGuardTests(unittest.TestCase):
    def node(self):
        n = adapter.FlightNode.__new__(adapter.FlightNode)
        n.c, n.s = Config(), healthy()
        n.mission = Mission(n.c)
        n.fc_system_id = n.fc_component_id = 1
        n.previous_odometry = None
        n.telemetry = {}
        n.receipt = lambda *args: 1.0
        n.journal = NS(append=lambda _: None)
        return n

    def test_wire_decode_ned_to_enu_unknown_quality_and_nan_cross_covariance(self):
        o = decode_odometry(packet(), 1, 1)
        self.assertEqual(o.position_enu, (3, 2, 4))
        self.assertAlmostEqual(o.velocity_enu[2], .3, places=6)
        self.assertAlmostEqual(o.position_sigma_m, .1, places=6)

    def test_wrong_fc_bad_frame_and_missing_extensions_never_accepted(self):
        for field, value in [('sysid', 2), ('compid', 190), ('framing_status', 2),
                             ('magic', 254), ('len', 230), ('msgid', 32)]:
            with self.subTest(field=field):
                p = packet(); setattr(p, field, value)
                self.assertIsNone(decode_odometry(p, 1, 1))
        p = packet(); p.payload64 = p.payload64[:2]
        self.assertIsNone(decode_odometry(p, 1, 1))

    def test_failed_quality_and_invalid_covariance_never_become_healthy(self):
        p = packet(length=233)
        payload = bytearray(struct.pack('<30Q', *p.payload64))
        payload[232] = 255  # int8 quality = -1.
        p.payload64 = list(struct.unpack('<30Q', payload))
        self.assertIsNone(decode_odometry(p, 1, 1))
        for value in (0, -1, math.nan):
            payload[232] = 0
            struct.pack_into('<f', payload, 60, value)
            p.payload64 = list(struct.unpack('<30Q', payload))
            self.assertTrue(math.isinf(decode_odometry(p, 1, 1).position_sigma_m))

    def test_reset_is_received_without_a_yaw_innovation_or_gyro(self):
        n = self.node()
        n.on_fc_mavlink(packet())
        n.mission.state = 'ASCEND'
        n.on_fc_mavlink(packet(counter=3, timestamp=1050000))
        self.assertTrue(n.s.fc_reset)
        n.on_fc_mavlink(packet(counter=3, timestamp=1100000))
        self.assertTrue(n.s.fc_reset)  # Latched; never rebase and continue.

    def test_counter_wrap_is_a_reset_and_preflight_resets_are_baselined(self):
        n = self.node()
        n.on_fc_mavlink(packet(counter=255))
        n.on_fc_mavlink(packet(counter=0, timestamp=1050000))
        self.assertFalse(n.s.fc_reset)
        n.mission.state = 'ADVANCE'
        n.on_fc_mavlink(packet(counter=1, timestamp=1100000))
        self.assertTrue(n.s.fc_reset)

    def test_duplicate_packets_do_not_refresh_and_source_rollback_latches(self):
        n = self.node(); calls = []
        n.receipt = lambda *args: calls.append(args) or 1
        n.on_fc_mavlink(packet()); n.on_fc_mavlink(packet())
        self.assertEqual(len(calls), 1)
        n.mission.state = 'ASCEND'
        n.on_fc_mavlink(packet(timestamp=900000))
        self.assertTrue(n.s.fc_reset)

    def test_ground_restart_before_start_accepts_new_epoch(self):
        n = self.node(); n.on_fc_mavlink(packet())
        n.on_fc_mavlink(packet(timestamp=10000))
        self.assertEqual(n.previous_odometry.timestamp_us, 10000)
        self.assertFalse(n.s.fc_reset)

    def test_gps_unknown_covariance_zero_variance_and_negative_matrix_rejected(self):
        n = self.node()
        msg = NS(status=NS(status=0, service=1), latitude=37, longitude=126, altitude=5,
                 position_covariance_type=0, position_covariance=[.04, 0, 0, 0, .04, 0, 0, 0, .04])
        n.on_gps(msg); self.assertTrue(math.isinf(n.s.gps_sigma_m))
        msg.position_covariance_type = 2
        n.on_gps(msg); self.assertAlmostEqual(n.s.gps_sigma_m, .2)
        msg.position_covariance = [0]*9
        n.on_gps(msg); self.assertTrue(math.isinf(n.s.gps_sigma_m))
        self.assertTrue(math.isinf(horizontal_sigma(1, 2, 1)))
        self.assertAlmostEqual(horizontal_sigma(1, .5, 1), math.sqrt(1.5))

    def test_legacy_const_flag_accepts_fresh_ground_only_and_gps_glitch_blocks(self):
        n = self.node()
        msg = NS(attitude_status_flag=True, velocity_horiz_status_flag=True, velocity_vert_status_flag=True,
                 accel_error_status_flag=False, const_pos_mode_status_flag=True,
                 pos_horiz_rel_status_flag=True, pos_horiz_abs_status_flag=True, gps_glitch_status_flag=False)
        n.on_estimator(msg)
        self.assertFalse(n.mission.health_fault(n.s))
        n.s.landed = False
        self.assertIn('constant-position', n.mission.health_fault(n.s))
        n.s.landed = True; n.s.landed_age = 3
        self.assertIn('constant-position', n.mission.health_fault(n.s))
        msg.const_pos_mode_status_flag = False; msg.gps_glitch_status_flag = True
        n.on_estimator(msg)
        self.assertFalse(n.s.estimator_valid)


class MissionGuardTests(unittest.TestCase):
    def test_missing_odometry_bad_covariance_and_failed_trial_gnss_prevent_start(self):
        for field, value in [('odometry_age', math.inf), ('fc_reset_counter', -1),
                             ('position_sigma_m', 1.8), ('velocity_sigma_mps', .5),
                             ('gps_sigma_m', math.sqrt(3.179089)), ('gps_age', 3), ('yaw_rate_rps', math.nan)]:
            s = healthy(); setattr(s, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError): Mission(Config()).start(0, s, 0)

    def test_inflight_reset_stale_telemetry_and_uncertainty_stop_normal_completion(self):
        for field, value in [('fc_reset', True), ('odometry_age', 1), ('velocity_sigma_mps', .4),
                             ('gps_sigma_m', 2), ('estimator_const_pos', True)]:
            h = Harness(); h.to_state('ADVANCE'); setattr(h.s, field, value)
            cmd = h.tick()
            self.assertEqual(cmd.request, 'LAND')
            self.assertEqual(h.m.result, 'aborted')
            self.assertFalse(h.m.zero_hold_completed)
            if field in {'fc_reset', 'odometry_age', 'velocity_sigma_mps', 'estimator_const_pos'}:
                self.assertFalse(cmd.publish)

    def test_persistent_yaw_excursion_aborts_but_short_transient_recovers(self):
        h = Harness(); h.to_state('ADVANCE'); h.s.yaw = math.radians(24)
        for _ in range(6): h.tick()
        self.assertEqual(h.m.state, 'ADVANCE')
        h.s.yaw = 0; h.tick(); h.s.yaw = math.radians(24)
        for _ in range(12): cmd = h.tick()
        self.assertEqual(h.m.state, 'LANDING')
        self.assertIn('sustained heading', h.m.reason)
        self.assertFalse(cmd.publish)

    def test_excessive_yaw_rate_aborts_even_inside_angle_tolerance(self):
        h = Harness(); h.s.height_m = .4; h.to_state('ASCEND'); h.s.yaw_rate_rps = .8
        for _ in range(12): cmd = h.tick()
        self.assertIn('sustained heading', h.m.reason)
        self.assertFalse(cmd.publish)

    def test_passive_xy_guard_does_not_add_correction(self):
        h = Harness(); h.s.height_m = .4; h.to_state('ASCEND')
        h.s.position_enu = (1.2, 0, 0)
        h.m.c = replace(h.m.c, position_velocity_residual_limit_m=100)
        for i in range(12):
            h.now += .05; h.s.odometry_stamp_s = h.now
            cmd = h.m.step(h.now, h.s)
            if cmd.publish: h.m.record_publish(cmd, h.now)
            self.assertEqual((cmd.vx, cmd.vy), (0, 0))
        self.assertIn('departed', h.m.reason)
        self.assertEqual(h.m.result, 'aborted')

    def test_vertical_disagreement_persists_before_abort(self):
        h = Harness(); h.s.height_m = .4; h.to_state('ASCEND')
        h.s.height_rate_mps = -.3; h.s.velocity_enu = (0, 0, .3)
        for _ in range(17):
            h.now += .05; h.s.odometry_stamp_s = h.now
            cmd = h.m.step(h.now, h.s)
            if cmd.publish: h.m.record_publish(cmd, h.now)
        self.assertIn('vertical velocity disagreement', h.m.reason)
        self.assertFalse(cmd.publish)

    def test_estimated_motion_excursion_restarts_hold_without_position_control(self):
        h = Harness(); h.to_state('ZERO_VELOCITY_HOLD')
        first = h.m.hold_since
        for _ in range(40): h.tick()
        h.s.velocity_enu = (.25, 0, 0); h.now += .05; h.s.odometry_stamp_s = h.now
        cmd = h.m.step(h.now, h.s); h.m.record_publish(cmd, h.now)
        self.assertIsNone(h.m.hold_since)
        self.assertEqual((cmd.vx, cmd.vy), (0, 0))
        h.tick()
        self.assertGreater(h.m.hold_since, first)
        for _ in range(80): h.tick()
        self.assertEqual(h.m.state, 'ZERO_VELOCITY_HOLD')

    def test_pilot_takeover_wins_over_new_guard_fault_and_never_reenters(self):
        h = Harness(); h.to_state('ADVANCE')
        h.s.mode = 'AUTO.LAND'; h.s.fc_reset = True
        cmd = h.tick()
        self.assertEqual(h.m.state, 'RELEASED')
        self.assertFalse(cmd.publish or cmd.request)

    def test_missing_zero_publications_cannot_complete_a_hold(self):
        h = Harness(); h.to_state('ZERO_VELOCITY_HOLD')
        for _ in range(6): cmd = h.tick(publish=False)
        self.assertEqual(h.m.result, 'aborted')
        self.assertIn('publication gap', h.m.reason)
        self.assertFalse(cmd.publish)
        self.assertFalse(h.m.zero_hold_completed)


class EvidenceReplayTests(unittest.TestCase):
    def test_real_oct02_signals_reject_start_and_trigger_isolated_guards(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
        from replay_failure_guards import replay
        result = replay()
        self.assertTrue(all(case['detected'] for case in result['cases'].values()))
        self.assertFalse(result['full_aircraft_or_flight_validation'])
