import unittest
from dataclasses import replace
import numpy as np
from we_meet_vio.mission import Mission,Parameters,Observation
from offline_simulation import sample,simulate


class ControlTests(unittest.TestCase):
    def running(self,state="APPROACH",position=(0,0,3)):
        m=Mission()
        ok,_=m.start(100,sample(100,armed=False,mode="POSCTL"))
        self.assertTrue(ok)
        m.state=state
        m.entered=100.
        m.last_tick=100.
        m.had_offboard=True
        s=sample(100.025,position)
        return m,s

    def test_full_mission_in_wind(self):
        result=simulate()
        self.assertTrue(result["success"],result)
        self.assertGreaterEqual(result["hold_elapsed_s"],5)
        for state in ("OFFBOARD","ARM","TAKEOFF","STABILIZE","APPROACH","BLEND","ALIGN","HOLD","LAND","DONE"):
            self.assertIn(state,result["states"])

    def test_full_mission_uses_initial_heading(self):
        for heading in (0.,1.57,-2.8):
            result=simulate(heading=heading)
            self.assertTrue(result["success"],result)
            self.assertLess(result["final_xy_error_m"],.12)

    def test_no_motion_does_not_count_command_distance(self):
        m,s=self.running()
        for i in range(1,1200):
            now=100+i*.025
            s=sample(now,[0,0,3])
            out=m.step(now,s)
            self.assertNotEqual(out.state,"DONE")
            self.assertFalse(out.completed_hold)
            self.assertAlmostEqual(m.goal[0],5)
        self.assertEqual(m.state,"APPROACH")

    def test_approach_timeout_without_motion(self):
        m,s=self.running()
        for i in range(1,1500):
            out=m.step(100+i*.025,sample(100+i*.025,[0,0,3]))
            if m.state=="LAND":
                break
        self.assertIn("timeout",out.reason)

    def test_goal_without_panel_is_failure(self):
        m,s=self.running(position=(4.92,0,3))
        out=m.step(100.025,s)
        self.assertEqual(out.state,"LAND")
        self.assertFalse(out.completed_hold)

    def test_handover_keeps_origin_heading_command(self):
        m,s=self.running(position=(4.3,0,3))
        m.command=np.array([.3,0,0])
        before=m.origin.copy()
        goal=m.goal.copy()
        s.observation=Observation(100.025,7,np.array([5.,0,0]),0.,-.3,3)
        out=m.step(100.025,s)
        self.assertEqual(out.state,"BLEND")
        np.testing.assert_array_equal(before,m.origin)
        np.testing.assert_array_equal(goal,m.goal)
        self.assertLessEqual(np.linalg.norm(out.velocity[:2]-[.3,0]),m.p.accel*.025+1e-9)

    def test_camera_timeout_even_if_imu_odometry_fresh(self):
        m,s=self.running()
        s.update_stamp=99.
        out=m.step(100.025,s)
        self.assertEqual(out.state,"LAND")
        self.assertIn("visual correction",out.reason)

    def test_reject_ground_start_armed_or_not_landed(self):
        for changes in ({"armed":True},{"landed":False},{"connected":False}):
            m=Mission()
            s=sample(100,armed=False,mode="POSCTL")
            for key,value in changes.items():
                setattr(s,key,value)
            self.assertFalse(m.start(100,s)[0])

    def test_sensor_stale_aborts(self):
        for field in ("height_stamp","image_stamp","detector_stamp","state_stamp","extended_state_stamp"):
            m,s=self.running()
            setattr(s,field,97.)
            self.assertEqual(m.step(100.025,s).state,"LAND",field)

    def test_high_covariance_and_unknown_covariance(self):
        for variance in (0.,1.,float("nan")):
            m,s=self.running()
            s.pose.pose_cov=np.eye(6)*variance
            self.assertEqual(m.step(100.025,s).state,"LAND")

    def test_position_reset(self):
        m,s=self.running()
        m.step(100.025,s)
        out=m.step(100.05,sample(100.05,[1.,0,3]))
        self.assertEqual(out.state,"LAND")
        self.assertIn("reset",out.reason)

    def test_invalid_fc_orientation_aborts_without_throwing(self):
        m,s=self.running()
        s.fc_pose=replace(s.fc_pose,q=np.zeros(4))
        out=m.step(100.025,s)
        self.assertEqual(out.state,"LAND")
        self.assertIn("orientation",out.reason)

    def test_clock_gap(self):
        m,s=self.running()
        out=m.step(100.5,sample(100.5,[0,0,3]))
        self.assertEqual(out.state,"LAND")
        self.assertIn("clock",out.reason)

    def test_manual_takeover_releases_without_land_request(self):
        m,s=self.running()
        s.mode="ALTCTL"
        out=m.step(100.025,s)
        self.assertEqual(out.state,"FAILED")
        self.assertFalse(out.publish)
        self.assertFalse(out.request_mode)

    def test_lidar_surface_jump_is_not_followed(self):
        m,s=self.running()
        for i in range(1,30):
            now=100+i*.025
            s=sample(now,[0,0,3])
            s.height=2.4
            out=m.step(now,s)
        self.assertEqual(out.state,"LAND")
        self.assertIn("inconsistent",out.reason)

    def test_fc_velocity_disagreement_aborts(self):
        m,s=self.running()
        for i in range(1,30):
            now=100+i*.025
            s=sample(now,[0,0,3])
            s.fc_pose=replace(s.fc_pose,v=np.array([.5,0,0]))
            out=m.step(now,s)
        self.assertEqual(out.state,"LAND")

    def test_panel_loss_brakes_then_lands(self):
        m,s=self.running(state="ALIGN")
        m.target=1
        m.anchor=np.array([0.,0,3])
        m.command[:]=[.1,0,0]
        for i in range(1,90):
            now=100+i*.025
            out=m.step(now,sample(now,[0,0,3]))
        self.assertEqual(out.state,"LAND")
        self.assertIn("lost",out.reason)

    def test_cannot_jump_to_other_panel(self):
        m,s=self.running(state="ALIGN")
        m.target=1
        m.anchor=np.array([0.,0,3])
        s.observation=Observation(100.025,2,np.array([0,0,0]),0.,0.,4)
        out=m.step(100.025,s)
        self.assertEqual(out.state,"ALIGN")
        self.assertIsNotNone(m.lost_since)
        self.assertIsNone(m.hold_since)

    def test_hold_requires_continuous_fresh_central_low_speed(self):
        for fault in ("centre","speed","height","yaw","stale"):
            m,s=self.running(state="ALIGN",position=(5,0,3))
            m.target=1
            m.anchor=np.array([5.,0,3])
            for i in range(1,120):
                now=100+i*.025
                s=sample(now,[5,0,3])
                s.observation=Observation(now,1,np.array([5,0,0]),0.,0.,4)
                out=m.step(now,s)
            self.assertEqual(out.state,"HOLD")
            if fault=="centre": s.observation.ex=.2
            if fault=="speed": s.pose.v=np.array([.11,0,0])
            if fault=="height": s.height=2.82
            if fault=="yaw":
                from we_meet_vio.geometry import quaternion,yaw_rotation
                s.pose.q=quaternion(yaw_rotation(.12))
            if fault=="stale": s.observation.stamp=now-.4
            s.pose.stamp=s.fc_pose.stamp=now+.025
            s.height_stamp=s.update_stamp=s.image_stamp=s.detector_stamp=now+.025
            s.state_stamp=s.extended_state_stamp=now+.025
            out=m.step(now+.025,s)
            self.assertFalse(out.completed_hold,fault)
            self.assertIsNone(m.hold_since,fault)

    def test_land_ack_stops_commands_and_requires_ground_disarm(self):
        m,s=self.running(state="LAND")
        s.mode="AUTO.LAND"
        out=m.step(100.025,s)
        self.assertEqual(out.state,"LAND")
        self.assertFalse(out.publish)
        s.armed=False
        s.landed=True
        m.hold_completed=True
        out=m.step(100.05,s)
        self.assertEqual(out.state,"DONE")

    def test_land_handover_timeout_is_failure_and_stops_stream(self):
        m,s=self.running(state="LAND")
        out=m.step(105.1,sample(105.1,[0,0,3]))
        self.assertEqual(out.state,"FAILED")
        self.assertFalse(out.publish)

    def test_parameter_validation(self):
        for change in ({"hold_s":4},{"forward_speed":1},{"distance_m":float("nan")},{"accel":-1},{"geofence_radius":4}):
            with self.assertRaises(ValueError):
                Parameters(**change)
