import unittest
import numpy as np
from we_meet_vio.geometry import (Pose,rotation,quaternion,yaw_rotation,Alignment,
                                 panel_intersection,corrected_height,PoseBuffer,covariance,transform)


def pose(stamp=100,p=(0,0,3),q=None,v=(0,0,0),omega=(0,0,0)):
    return Pose(stamp,np.array(p,dtype=float),np.array([0.,0,0,1] if q is None else q),
                np.array(v,dtype=float),np.array(omega,dtype=float),np.eye(6)*.01,np.eye(6)*.01)


class GeometryTests(unittest.TestCase):
    def test_quaternion_roundtrip_large_rotations(self):
        for angle in (0.,.4,1.57,3.14,-2.8):
            r=yaw_rotation(angle)
            np.testing.assert_allclose(rotation(quaternion(r)),r,atol=1e-12)

    def test_openvins_quaternion_is_not_conjugated(self):
        source=pose(q=quaternion(yaw_rotation(np.pi/2)),v=(0,1,0))
        a=Alignment(np.eye(4))
        a.latch(source,source)
        out=a.apply(source)
        np.testing.assert_allclose(out.r@np.array([1.,0,0]),[0,1,0],atol=1e-12)
        np.testing.assert_allclose(out.r.T@out.v,[1,0,0],atol=1e-12)

    def test_body_origin_and_lever_velocity(self):
        t=np.eye(4)
        t[:3,3]=[.1,0,0]
        source=pose(p=(1,0,3),v=(0,.1,0),omega=(0,0,1))
        fc=pose(p=(.9,0,3))
        a=Alignment(t)
        a.latch(source,fc)
        out=a.apply(source)
        np.testing.assert_allclose(out.p,fc.p)
        np.testing.assert_allclose(out.v,[0,0,0],atol=1e-12)
        self.assertGreater(np.diag(out.pose_cov)[1],.01)

    def test_world_heading_alignment_applied_once(self):
        a=Alignment(np.eye(4))
        a.latch(pose(p=(0,0,0)),pose(p=(10,20,0),q=quaternion(yaw_rotation(np.pi/2))))
        out=a.apply(pose(p=(1,0,0),v=(1,0,0)))
        np.testing.assert_allclose(out.p,[10,21,0],atol=1e-12)
        np.testing.assert_allclose(out.v,[0,1,0],atol=1e-12)

    def test_mount_rotation_imu_axes_to_body(self):
        t=np.eye(4)
        t[:3,:3]=yaw_rotation(np.pi/2)
        source=pose(q=quaternion(yaw_rotation(-np.pi/2)),v=(1,0,0),omega=(1,0,0))
        a=Alignment(t)
        a.latch(source,pose())
        out=a.apply(source)
        np.testing.assert_allclose(out.omega,[0,1,0],atol=1e-12)

    def test_downward_pixel_direction_and_camera_offset(self):
        t=np.eye(4)
        t[:3,:3]=[[0,-1,0],[-1,0,0],[0,0,-1]]
        t[:3,3]=[.07,.05,-.16]
        result=panel_intersection([0,-.1,1],pose(),t,0)
        np.testing.assert_allclose(result,[.354,.05,0],atol=1e-12)
        right=panel_intersection([.1,0,1],pose(),t,0)
        self.assertLess(right[1],0)

    def test_reject_upward_ray(self):
        with self.assertRaises(ValueError):
            panel_intersection([0,0,1],pose(),np.eye(4),0)

    def test_lidar_offset_applied_once(self):
        self.assertAlmostEqual(corrected_height(3,np.eye(3),[0,0,-.16]),3.16)

    def test_lidar_tilt_rejected(self):
        r=np.array([[1,0,0],[0,0,-1],[0,1,0.]])
        with self.assertRaises(ValueError):
            corrected_height(3,r,[0,0,0])

    def test_capture_time_pose_interpolation(self):
        buffer=PoseBuffer()
        buffer.add(pose(100,(0,0,3)))
        buffer.add(pose(100.05,(.05,0,3),quaternion(yaw_rotation(.1))))
        interpolated=buffer.at(100.025)
        np.testing.assert_allclose(interpolated.p,[.025,0,3])
        self.assertIsNone(buffer.at(99.9))
        self.assertIsNone(buffer.at(100.1))
        with self.assertRaises(ValueError):
            buffer.add(pose(99.9))

    def test_pose_gap_does_not_interpolate(self):
        buffer=PoseBuffer()
        buffer.add(pose(100))
        buffer.add(pose(100.2))
        self.assertIsNone(buffer.at(100.1))

    def test_invalid_covariance_and_nonrigid_transform(self):
        for c in (np.zeros((6,6)),np.eye(6)*float('nan'),-np.eye(6)):
            with self.assertRaises(ValueError): covariance(c)
        t=np.eye(4)
        t[0,0]=2
        with self.assertRaises(ValueError): transform(t)
