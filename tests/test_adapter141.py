"""Adapter integration with the actual archived parent methods, without ROS/FC."""
import math
import unittest
from tempfile import TemporaryDirectory
from pathlib import Path

from ros_doubles import FakeNode, Message
from da_daka_control.autonomous_cleaning_fsm import CleaningMissionState
from we_meet_prototype141.node import Flight141Mission
from test_141 import REF
from da_daka_control.panel_mapping import PanelTarget


def make_node(tmp_path):
    FakeNode.overrides = {'prototype_log_directory': str(tmp_path),
                          'configuration_approved': True,
                          'calibration_approved': True,
                          'localization_test_mode': True,
                          'localization_scan_yaw_aligned': True}
    return Flight141Mission()


class Adapter141Tests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.tmp_path = Path(self.temporary.name)

    def test_historical_route_enters_descent_and_uses_original_panel_three(self):
        node = make_node(self.tmp_path)
        try:
            node._fsm.transition(CleaningMissionState.SURVEY)
            node._fsm.survey_complete([PanelTarget(**p) for p in REF['panels_from_field_document']])
            node._launch_xyz = (*REF['scan']['launch_enu_xy'], 0.)
            node._pose_xyz = (*REF['scan']['launch_enu_xy'], 3.)
            node._tick_plan_route(0.)
            assert node._fsm.state == CleaningMissionState.DESCEND
            assert tuple(p.target.panel_id for p in node._fsm.panels) == (3, 1, 2, 6)
            assert node._fsm.current_panel.target.east_m == 1.675512
        finally:
            node.destroy_node()

    def test_pose_heading_change_keeps_prearm_yaw_without_v1_reset_abort(self):
        node = make_node(self.tmp_path)
        try:
            yaw = math.radians(-120.8676528930664)
            node._launch_yaw_rad = yaw
            node._armed = True
            node._fsm.transition(CleaningMissionState.TAKEOFF)
            changed_yaw = yaw + math.radians(3.1848758019551027)
            node._pose_cb(Message(pose=Message(
                position=Message(x=0., y=0., z=1.),
                orientation=Message(x=0., y=0., z=math.sin(changed_yaw/2),
                                    w=math.cos(changed_yaw/2)))))
            assert node._launch_yaw_rad == yaw
            assert node._fsm.state == CleaningMissionState.TAKEOFF
        finally:
            node.destroy_node()

    def test_velocity_journal_observes_exact_archived_vertical_command(self):
        node = make_node(self.tmp_path)
        try:
            node._distance_command = Message(twist=Message(
                linear=Message(z=1.5), angular=Message(z=-.05)))
            node._distance_command_time_s = 10.
            node._publish_combined_velocity(10.1, include_visual=False)
            sent = node.publishers['/mavros/setpoint_velocity/cmd_vel'].messages[-1]
            assert sent.twist.linear.x == 0.
            assert sent.twist.linear.y == 0.
            assert sent.twist.linear.z == 1.5
            assert sent.twist.angular.z == -.05
        finally:
            node.destroy_node()

    def test_home_reconstruction_reaches_live_parent_setpoint_and_does_not_move_yaw(self):
        node = make_node(self.tmp_path)
        try:
            node._pose_xyz = (3.073, 1.037, 0.)
            node._home_xy = (3.284, .921)
            node._reset_mission_bookkeeping()
            node._launch_xyz = (3.073, 1.037, 0.)
            node._launch_yaw_rad = math.radians(-120.8676528930664)
            node._home_xy = (2.975919246673584, .9730047583580017)
            assert node._refresh_historical_home_anchor()
            assert abs(node._launch_xyz[0] - 2.7649991512298584) < .001
            assert abs(node._launch_xyz[1] - 1.0895180702209473) < .001
            node._publish_position_hold((*node._launch_xyz[:2], 3.))
            sent = node.publishers['/mavros/setpoint_position/local'].messages[-1]
            assert sent.header.frame_id == 'map'
            assert sent.pose.position.x == node._launch_xyz[0]
            assert sent.pose.position.y == node._launch_xyz[1]
            assert sent.pose.orientation.z == math.sin(node._launch_yaw_rad / 2)
            assert node._launch_yaw_rad == math.radians(-120.8676528930664)
        finally:
            node.destroy_node()


    def test_old_takeoff_handover_to_survey_is_retained(self):
        node = make_node(self.tmp_path)
        try:
            node._fsm.transition(CleaningMissionState.TAKEOFF)
            node._stage = 'EXIT_OFFBOARD'
            node._takeoff_enabled = False
            node._mode = 'AUTO.LOITER'
            node._request_bool = lambda *a: None
            node._request_mode = lambda *a: None
            node._publish_combined_velocity = lambda *a, **k: None
            node._tick_takeoff(10.)
            assert node._fsm.state == CleaningMissionState.SURVEY
            assert node._stage == 'HANDOVER'
            assert node._survey_started_s is None
        finally:
            node.destroy_node()


    def test_archived_survey_generates_reference_geometry_after_launch_hold(self):
        node = make_node(self.tmp_path)
        try:
            node._fsm.transition(CleaningMissionState.SURVEY)
            node._ai_healthy = True
            node._mode = 'OFFBOARD'
            node._control_kind, node._stage = 'position', 'ACTIVE'
            node._launch_xyz = (2.7649991512298584, 1.0895180702209473, 0.)
            node._launch_yaw_rad = math.radians(-120.8676528930664)
            node._pose_xyz = (*node._launch_xyz[:2], 3.)
            node._yaw_rad = node._launch_yaw_rad
            node._distance_m = 3.
            node._advance_and_publish_position = lambda *a: None
            for step in range(61):
                now = 10. + step * .05
                node._survey_home_speed_filter.update(0., now)
                node._tick_survey(now)
                if node._survey_started_s is not None:
                    break
            assert len(node._localization_scan_waypoints) == 5
            assert abs(node._localization_scan_waypoints[0][0] - 1.9905238151550293) < 1e-6
            assert 12.3 <= node._survey_started_s <= 12.5
        finally:
            node.destroy_node()


    def test_large_historical_home_shift_fails_instead_of_crashing(self):
        node = make_node(self.tmp_path)
        try:
            node._pose_xyz = (0., 0., 0.)
            node._home_xy = (0., 0.)
            node._reset_mission_bookkeeping()
            node._launch_xyz = (0., 0., 0.)
            node._home_xy = (11., 0.)
            assert not node._refresh_historical_home_anchor()
            assert node._fsm.state == CleaningMissionState.ABORT
        finally:
            node.destroy_node()

    def test_retained_home_reference_does_not_expire_but_live_gps_does(self):
        node = make_node(self.tmp_path)
        try:
            node._pose_xyz = (3., 2., 0.)
            node._home_xy = (3., 2.)
            node._home_latlon = node._gps_latlon = (37., 127.)
            node._home_time = 1.
            node._gps_time = 99.9
            assert node._reference_failure(100.) is None
            assert node._reference_failure(107.) == 'GPS reference data stale'
        finally:
            node.destroy_node()
