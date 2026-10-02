"""Regression against raw 141 geometry and the recorded pre-arm Home data."""
import json
import math
from pathlib import Path

import unittest
import ros_doubles

from da_daka_control.survey_planner import rectangular_scan_waypoints
from da_daka_control.route_planner import plan_panel_route
from da_daka_control.panel_mapping import PanelTarget
from we_meet_prototype141.history import LegacyHomeAnchor, home_crosscheck_error, historical_panel_order

ROOT = Path(__file__).resolve().parents[1]
REF = json.loads((ROOT / 'evidence/flight141_reference.json').read_text())


class Reference141Tests(unittest.TestCase):
    def test_141_all_five_waypoints_match_original_fc_targets(self):
        scan = REF['scan']
        actual = rectangular_scan_waypoints(
            *scan['launch_enu_xy'], scan['width_m'], scan['depth_m'],
            yaw_rad=math.radians(scan['yaw_enu_deg']),
            forward_offset_m=scan['forward_offset_m'],
            lateral_offset_m=scan['lateral_offset_m'])
        for got, expected in zip(actual, scan['waypoints_enu_xy']):
            assert math.dist(got, expected) < 1e-6


    def test_home_follow_reproduces_both_raw_home_events_and_final_scan_anchor(self):
        start = REF['request_reference_from_field_document']
        anchor = LegacyHomeAnchor(start['enu_xy'], start['home_enu_xy'])
        for event in REF['home_xy_events']:
            target = anchor.target_xy(event['enu_xy'])
            self.assertLess(math.dist(target, [h+off for h, off in zip(event['enu_xy'], anchor.offset_xy)]), 1e-12)
        assert math.dist(target, REF['scan']['launch_enu_xy']) < .001
        # Home moves the reference even though the original local origin did not.
        self.assertAlmostEqual(math.dist(target, start['enu_xy']), .313, delta=.002)


    def test_141_four_cluster_route_preserves_old_ids(self):
        targets = [PanelTarget(**values) for values in REF['panels_from_field_document']]
        xy = tuple(REF['scan']['launch_enu_xy'])
        assert historical_panel_order(xy, targets, xy) == (3, 1, 2, 6)
        # Explicitly expose the supplied snapshot / actual-flight discrepancy.
        assert plan_panel_route(xy, targets, xy).panel_ids == (6, 2, 1, 3)


    def test_historical_gps_home_offset_crosscheck_is_not_a_position_command(self):
        assert home_crosscheck_error((3., 2.), (3., 2.), (37., 127.), (37., 127.)) == 0
        assert home_crosscheck_error((5., 2.), (3., 2.), (37., 127.), (37., 127.)) == 2.
        with self.assertRaises(ValueError):
            LegacyHomeAnchor((0, 0), (0, 0)).target_xy((11, 0))
