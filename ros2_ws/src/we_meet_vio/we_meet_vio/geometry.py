"""ROS ENU world, FLU body, optical camera (right/down/forward); metres/radians."""
from dataclasses import dataclass
import math
from collections import deque
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1) > .05:
        raise ValueError("invalid unit quaternion xyzw")
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(r):
    # Symmetric eigensystem avoids division near a 180-degree rotation.
    r = np.asarray(r)
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3
    _, vecs = np.linalg.eigh(k)
    q = vecs[:, -1]
    return q if q[3] >= 0 else -q


def yaw(r):
    return math.atan2(r[1, 0], r[0, 0])


def yaw_rotation(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0], [s,c,0], [0,0,1.]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def transform(t, name="transform"):
    t = np.asarray(t, dtype=float)
    if t.shape != (4,4) or not np.all(np.isfinite(t)):
        raise ValueError(name + " must be a finite 4x4 matrix")
    if not np.allclose(t[3], [0,0,0,1]) or not np.allclose(t[:3,:3].T@t[:3,:3], np.eye(3), atol=1e-5) or np.linalg.det(t[:3,:3]) < .999:
        raise ValueError(name + " must be a rigid right-handed transform")
    return t


def skew(a):
    x,y,z = a
    return np.array([[0,-z,y], [z,0,-x], [-y,x,0]])


def covariance(c):
    c = np.asarray(c, dtype=float).reshape(6,6)
    if not np.all(np.isfinite(c)) or not np.allclose(c,c.T,atol=1e-5) or np.min(np.linalg.eigvalsh(c)) < -1e-7 or np.any(np.diag(c) <= 0):
        raise ValueError("unknown/invalid covariance; zero covariance is not perfect certainty")
    return c


@dataclass
class Pose:
    stamp: float
    p: np.ndarray
    q: np.ndarray
    v: np.ndarray  # WORLD linear velocity, unlike nav_msgs/Odometry twist
    omega: np.ndarray  # BODY angular velocity
    pose_cov: np.ndarray
    twist_cov: np.ndarray

    @property
    def r(self):
        return rotation(self.q)


class Alignment:
    """Latch while disarmed. Convert OpenVINS IMU-origin pose to body-origin ROS map.

    Upstream odomimu xyzw already equals the ItoG Hamilton quaternion: do NOT conjugate.
    Upstream twist is IMU-local. T_body_imu maps IMU vectors/points into body FLU.
    """
    def __init__(self, t_body_imu):
        self.t = transform(t_body_imu, "T_body_imu")
        self.a = None
        self.b = None

    def latch(self, vio: Pose, fc: Pose):
        r_gb = vio.r @ self.t[:3,:3].T
        self.a = yaw_rotation(wrap(yaw(fc.r)-yaw(r_gb)))
        r_wb = self.a @ r_gb
        p_body_g = vio.p - r_gb @ self.t[:3,3]
        self.b = fc.p-self.a@p_body_g

    def apply(self, vio: Pose):
        if self.a is None:
            raise ValueError("alignment not latched")
        r_gb = vio.r@self.t[:3,:3].T
        r_wb = self.a@r_gb
        r_bi = self.t[:3,:3]
        lever = self.t[:3,3]
        omega_b = r_bi@vio.omega
        # Input v is global here; remove lever-arm velocity at the IMU origin.
        v_body_g = vio.v - r_gb@np.cross(omega_b, lever)
        pc = covariance(vio.pose_cov)
        vc = covariance(vio.twist_cov)
        # OpenVINS orientation error is IMU-local. Use a conservative spectral
        # bound before applying the lever-arm Jacobian, including correlations.
        # This does not claim zero covariance or unmeasured centimetre accuracy.
        jp = np.eye(6)
        jp[:3,3:] = r_wb@skew(lever)
        jp[3:,3:] = r_wb
        pc_out = float(np.max(np.linalg.eigvalsh(pc)))*(jp@jp.T)
        jt = np.zeros((6,6))
        jt[:3,:3] = r_bi
        jt[:3,3:] = skew(lever)@r_bi
        jt[3:,3:] = r_bi
        vc_out = jt@vc@jt.T
        return Pose(vio.stamp, self.a@vio.p+self.b-r_wb@lever,
                    quaternion(r_wb), self.a@v_body_g, omega_b, pc_out, vc_out)


class PoseBuffer:
    def __init__(self, horizon_s=2.0):
        self.data = deque()
        self.horizon = horizon_s

    def add(self, p):
        if self.data and p.stamp <= self.data[-1].stamp:
            raise ValueError("pose time reversed or repeated")
        self.data.append(p)
        while self.data and p.stamp-self.data[0].stamp > self.horizon:
            self.data.popleft()

    def at(self, stamp, max_gap=.08):
        # Interpolate bracketing samples only; never assign current pose to old images.
        for a,b in zip(self.data, list(self.data)[1:]):
            if a.stamp <= stamp <= b.stamp and b.stamp-a.stamp <= max_gap:
                t=(stamp-a.stamp)/(b.stamp-a.stamp)
                qb = b.q if np.dot(a.q,b.q) >= 0 else -b.q
                q=(1-t)*a.q+t*qb
                q=q/np.linalg.norm(q)
                return Pose(stamp,(1-t)*a.p+t*b.p,q,(1-t)*a.v+t*b.v,
                            (1-t)*a.omega+t*b.omega,a.pose_cov,a.twist_cov)
        return None


def panel_intersection(ray_optical, pose, t_body_camera, plane_z):
    t = transform(t_body_camera)
    origin = pose.p+pose.r@t[:3,3]
    ray = pose.r@t[:3,:3]@np.asarray(ray_optical)
    if ray[2] >= -.2:
        raise ValueError("camera ray does not intersect a downward horizontal panel")
    distance=(plane_z-origin[2])/ray[2]
    if not 0 < distance < 30:
        raise ValueError("panel plane behind camera or too far")
    return origin+distance*ray


def corrected_height(distance, body_r, sensor_position_body):
    # Down-looking sensor aligned with -body Z. Offset applied once.
    cosine=body_r[2,2]
    if not .90 <= cosine <= 1.00001 or not .2 <= distance <= 8:
        raise ValueError("invalid LiDAR range/tilt")
    return distance*cosine-(body_r@np.asarray(sensor_position_body))[2]
