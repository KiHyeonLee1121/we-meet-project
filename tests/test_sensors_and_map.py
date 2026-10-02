import unittest
import numpy as np
import cv2
from we_meet_vio.health import StreamGuard,ClockGuard
from we_meet_vio.lidar import Parser
from we_meet_vio.work_map import WorkMap
from we_meet_vio.perception import detect,diagonal_centre,optical_ray


class SensorTests(unittest.TestCase):
    def test_rate_and_stale_source(self):
        g=StreamGuard(180,.08,.025)
        for i in range(420):
            now=100+i*.005
            self.assertTrue(g.feed(now,now+.01))
        self.assertTrue(g.ready(now+.01))
        self.assertFalse(g.ready(now+.2))
        self.assertFalse(g.feed(now+1,now+.01))

    def test_low_rate_cannot_be_relabelled_high_rate(self):
        g=StreamGuard(180,.08,.05)
        for i in range(150):
            now=100+i*.02
            g.feed(now,now+.01)
        self.assertFalse(g.ready(now+.01))

    def test_imu_timestamp_repetition_and_clock_step(self):
        g=StreamGuard(1,.5,1.)
        self.assertTrue(g.feed(100,100.1))
        self.assertFalse(g.feed(100,100.2))
        clock=ClockGuard()
        self.assertTrue(clock.check(100,1))
        self.assertTrue(clock.check(100.1,1.1))
        self.assertFalse(clock.check(100.5,1.2))
        self.assertFalse(clock.check(100.6,1.3))

    def test_tfluna_partial_checksum_and_recovery(self):
        data=bytearray([0x59,0x59,0x2c,1,200,0,0,8])
        data.append(sum(data)&255)
        parser=Parser()
        self.assertEqual(parser.feed(b'garbage'+data[:4]),[])
        frames=parser.feed(data[4:])
        self.assertEqual(frames[0].distance,3.)
        self.assertEqual(frames[0].strength,200)
        bad=bytearray(data)
        bad[-1]^=1
        recovered=parser.feed(bad+data)
        self.assertEqual(len(recovered),1)
        self.assertGreater(parser.errors,0)

    def test_parser_memory_bounded_on_noise(self):
        parser=Parser()
        parser.feed(b'\x11'*100000)
        self.assertLessEqual(len(parser.buffer),1)


class MapTests(unittest.TestCase):
    def test_distinct_frame_acquisition_and_target_lock(self):
        m=WorkMap()
        item=(np.array([5.,0.,0]),.1,0.,.9)
        panel=m.observe(100,[item])[0][0]
        self.assertEqual(m.observe(100,[item]),[])
        for stamp in (100.12,100.25): m.observe(stamp,[item])
        self.assertEqual(panel.hits,3)
        m.select(panel.id)
        other=m.observe(100.37,[(np.array([7.,0,0]),0.,0.,.9)])[0][0]
        with self.assertRaises(ValueError): m.select(other.id)
        m.complete()
        self.assertEqual(panel.state,"held_5s")

    def test_acquisition_gap_resets_hits(self):
        m=WorkMap()
        item=(np.array([5.,0,0]),0.,0.,.9)
        m.observe(100,[item])
        panel=m.observe(101,[item])[0][0]
        self.assertEqual(panel.hits,1)

    def test_two_candidates_cannot_share_id_in_one_frame(self):
        m=WorkMap()
        outputs=m.observe(100,[(np.array([5.,0,0]),0,0,.9),(np.array([5.2,0,0]),0,0,.9)])
        self.assertNotEqual(outputs[0][0].id,outputs[1][0].id)


class PerceptionTests(unittest.TestCase):
    def test_dark_rectangle_original_coordinates(self):
        image=np.full((480,640),230,np.uint8)
        cv2.rectangle(image,(200,150),(400,320),40,-1)
        candidates=detect(image)
        self.assertEqual(len(candidates),1)
        np.testing.assert_allclose(candidates[0]["center"],[300,235],atol=1)
        self.assertEqual(candidates[0]["width"],640)

    def test_blank_and_clipped_not_panels(self):
        image=np.full((480,640),230,np.uint8)
        self.assertEqual(detect(image),[])
        cv2.rectangle(image,(0,100),(200,300),40,-1)
        self.assertEqual(detect(image),[])

    def test_projective_center_not_corner_mean(self):
        corners=np.array([[0.,0],[4,0],[3,3],[1,3]])
        centre=diagonal_centre(corners)
        self.assertAlmostEqual(centre[0],2)
        self.assertAlmostEqual(centre[1],2)
        self.assertNotAlmostEqual(centre[1],corners.mean(axis=0)[1])

    def test_undistortion_uses_calibration(self):
        cal={"intrinsics":[500.,500.,320.,240.],"distortion":[0.,0.,0.,0.]}
        np.testing.assert_allclose(optical_ray([370,240],cal),[.1,0,1])
