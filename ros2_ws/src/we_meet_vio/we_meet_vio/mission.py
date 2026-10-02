"""Deterministic flight controller. No ROS, hardware, arming or external writes here."""
from dataclasses import dataclass, fields
import math
import numpy as np
from .geometry import Pose, yaw, wrap


@dataclass(frozen=True)
class Parameters:
    height_m: float=3.0
    distance_m: float=5.0
    hold_s: float=5.0
    forward_speed: float=.35
    visual_speed: float=.12
    accel: float=.25
    vertical_speed: float=.25
    vertical_accel: float=.4
    position_kp: float=.55
    velocity_kd: float=.2
    height_kp: float=.65
    height_kd: float=.15
    yaw_kp: float=1.0
    yaw_rate: float=.30
    center_half_width: float=.15  # fraction of full image width, 30% total box
    center_half_height: float=.15
    stable_s: float=2.0
    height_tolerance: float=.12
    hold_speed: float=.08
    yaw_tolerance: float=.0873
    max_yaw_error: float=.26
    cross_track_limit: float=.8
    geofence_radius: float=7.0
    max_height: float=3.7
    max_velocity: float=.9
    pose_timeout: float=.18
    visual_update_timeout: float=.3
    lidar_timeout: float=.2
    panel_timeout: float=.3
    panel_lost_land_s: float=2.0
    blend_s: float=1.5
    prestream_s: float=1.5
    command_timeout: float=5.0
    takeoff_timeout: float=25.0
    approach_timeout: float=35.0
    align_timeout: float=30.0
    max_flight_s: float=95.0
    land_handover_s: float=5.0
    land_timeout_s: float=35.0
    max_tick_gap: float=.15
    max_pose_sigma: float=.65
    max_velocity_sigma: float=.35
    fc_position_tolerance: float=.65
    fc_velocity_tolerance: float=.35
    range_vio_tolerance: float=.35

    def __post_init__(self):
        for f in fields(self):
            v=getattr(self,f.name)
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v <= 0:
                raise ValueError(f.name+" must be a positive finite number")
        if not 1 <= self.height_m <= 3.5 or not 1 <= self.distance_m <= 6:
            raise ValueError("single-panel test envelope: height 1..3.5m, distance 1..6m")
        if self.hold_s < 5 or max(self.center_half_width,self.center_half_height) > .3:
            raise ValueError("hold >=5s and central half-width/height <=30%")
        if self.visual_speed > self.forward_speed or self.forward_speed > .5 or self.vertical_speed > .4:
            raise ValueError("test speed limits exceeded")
        if self.geofence_radius <= self.distance_m or self.max_height <= self.height_m:
            raise ValueError("geofence must contain the mission goal")


@dataclass
class Observation:
    stamp: float
    panel_id: int
    xyz: np.ndarray
    ex: float  # centre error / FULL image width, original pixels
    ey: float
    hits: int


@dataclass
class Inputs:
    pose: Pose | None=None
    fc_pose: Pose | None=None
    height: float=0.
    height_stamp: float=0.
    update_stamp: float=0.  # OpenVINS poseimu: actual camera correction time
    image_stamp: float=0.
    detector_stamp: float=0.
    observation: Observation | None=None
    ready: bool=False
    reason: str="not ready"
    connected: bool=False
    armed: bool=False
    landed: bool=False
    mode: str=""
    state_stamp: float=0.
    extended_state_stamp: float=0.


@dataclass
class Output:
    velocity: np.ndarray
    yaw_rate: float=0.
    publish: bool=False
    request_mode: str=""
    request_arm: bool=False
    state: str="IDLE"
    reason: str=""
    completed_hold: bool=False


def limited(v, limit):
    n=np.linalg.norm(v)
    return v*(limit/n) if n>limit else v


class Mission:
    def __init__(self, p=Parameters()):
        self.p=p
        self.state="IDLE"
        self.reason=""
        self.started=self.entered=self.last_tick=None
        self.origin=self.goal=self.anchor=None
        self.heading=0.
        self.direction=None
        self.ground_z=None
        self.target=None
        self.hold_since=self.stable_since=self.lost_since=None
        self.mismatch_since=None
        self.command=np.zeros(3)
        self.hold_completed=False
        self.had_offboard=False
        self.last_pose=None
        self.success=False

    def change(self, state, now, reason=""):
        self.state=state
        self.entered=now
        if reason:
            self.reason=reason
        self.hold_since=None
        self.stable_since=None
        # DO NOT reset origin, heading, command or goal at camera handover.

    def start(self, now, s):
        if self.state != "IDLE":
            return False,"mission is one-shot; restart node after completion/abort"
        error=self.health(now,s)
        if error or not s.ready or s.armed or not s.landed:
            return False,error or s.reason or "must be landed and disarmed"
        self.origin=s.pose.p.copy()
        self.anchor=self.origin.copy()
        self.heading=yaw(s.pose.r)
        self.direction=np.array([math.cos(self.heading),math.sin(self.heading),0.])
        self.goal=self.origin+self.direction*self.p.distance_m
        self.ground_z=s.pose.p[2]-s.height
        self.started=now
        self.last_tick=now
        self.change("PRESTREAM",now)
        return True,"start accepted"

    def abort(self, now, reason="operator abort"):
        if self.state not in ("IDLE","DONE","FAILED","LAND"):
            self.change("LAND",now,reason)

    def health(self, now, s):
        p=self.p
        if not s.connected or now-s.state_stamp > 1.5 or now-s.extended_state_stamp > 2.5:
            return "FC connection/state stale"
        if s.pose is None or s.fc_pose is None:
            return "VIO / FC pose absent"
        for name,stamp,age in (("VIO",s.pose.stamp,p.pose_timeout),
                               ("FC pose",s.fc_pose.stamp,.3),
                               ("visual correction",s.update_stamp,p.visual_update_timeout),
                               ("camera",s.image_stamp,p.visual_update_timeout),
                               ("detector",s.detector_stamp,p.visual_update_timeout),
                               ("LiDAR",s.height_stamp,p.lidar_timeout)):
            if not -.02 <= now-stamp <= age:
                return name+" stale/future"
        for pose in (s.pose,s.fc_pose):
            if not np.all(np.isfinite(np.r_[pose.p,pose.q,pose.v,pose.omega])):
                return "nonfinite estimate"
        try:
            r=s.pose.r
            if not np.isfinite(s.pose.pose_cov).all() or not np.isfinite(s.pose.twist_cov).all():
                return "nonfinite covariance"
            if r[2,2] < math.cos(.35):
                return "excess tilt"
            if np.any(np.diag(s.pose.pose_cov)[:3] <= 0) or np.max(np.diag(s.pose.pose_cov)[:3]) > p.max_pose_sigma**2:
                return "VIO position covariance excessive/unknown"
            if np.any(np.diag(s.pose.twist_cov)[:3] <= 0) or np.max(np.diag(s.pose.twist_cov)[:3]) > p.max_velocity_sigma**2:
                return "VIO velocity covariance excessive/unknown"
        except ValueError:
            return "invalid orientation"
        if not math.isfinite(s.height) or not .15 < s.height < p.max_height:
            return "height out of envelope"
        if np.linalg.norm(s.pose.v) > p.max_velocity:
            return "excess estimated speed"
        if abs(wrap(yaw(s.pose.r)-yaw(s.fc_pose.r))) > .17:
            return "FC/VIO yaw disagreement"
        if not s.ready:
            return s.reason or "sensor / fusion readiness lost"
        return ""

    def step(self, now, s):
        p=self.p
        out=Output(np.zeros(3),state=self.state,reason=self.reason,completed_hold=self.hold_completed)
        if self.state in ("IDLE","DONE","FAILED"):
            return out
        dt=now-self.last_tick
        self.last_tick=now
        if dt <= 0 or dt > p.max_tick_gap:
            self.abort(now,"control clock jump/gap")

        # Manual mode change relinquishes control immediately; do not steal RC back.
        if self.had_offboard and s.mode not in ("OFFBOARD","AUTO.LAND"):
            self.change("FAILED",now,"FC mode/RC takeover: control released")
            return Output(np.zeros(3),state=self.state,reason=self.reason)
        if self.state == "LAND":
            return self.land(now,s)

        error=self.health(now,s)
        if error:
            self.abort(now,error)
            return self.land(now,s)
        if now-self.started > p.max_flight_s:
            self.abort(now,"mission timeout")
            return self.land(now,s)
        if self.origin is not None:
            displacement=s.pose.p-self.origin
            if np.linalg.norm(displacement[:2]) > p.geofence_radius or s.pose.p[2]-self.ground_z > p.max_height:
                self.abort(now,"VIO geofence exceeded")
                return self.land(now,s)
            if abs(wrap(yaw(s.pose.r)-self.heading)) > p.max_yaw_error:
                self.abort(now,"heading drift exceeded")
                return self.land(now,s)
        # Detect coordinate restart/jump despite MAVROS not carrying reset_counter.
        if self.last_pose is not None and s.pose.stamp > self.last_pose.stamp:
            ds=s.pose.stamp-self.last_pose.stamp
            if np.linalg.norm(s.pose.p-self.last_pose.p-self.last_pose.v*ds) > .25:
                self.abort(now,"VIO discontinuity/reset")
                return self.land(now,s)
        self.last_pose=s.pose
        inconsistency=(np.linalg.norm(s.pose.p[:2]-s.fc_pose.p[:2]) > p.fc_position_tolerance or
                       np.linalg.norm(s.pose.v-s.fc_pose.v) > p.fc_velocity_tolerance or
                       abs((s.pose.p[2]-self.ground_z)-s.height) > p.range_vio_tolerance)
        if inconsistency:
            if self.mismatch_since is None:
                self.mismatch_since=now
            elif now-self.mismatch_since > .5:
                self.abort(now,"FC/VIO/LiDAR inconsistent")
                return self.land(now,s)
        else:
            self.mismatch_since=None

        elapsed=now-self.entered
        mode=""
        arm=False
        desired=np.zeros(3)
        if self.state == "PRESTREAM":
            if elapsed >= p.prestream_s:
                self.change("OFFBOARD",now)
        elif self.state == "OFFBOARD":
            mode="OFFBOARD"
            if s.mode == "OFFBOARD":
                self.had_offboard=True
                self.change("ARM",now)
            elif elapsed > p.command_timeout:
                self.abort(now,"OFFBOARD not confirmed")
        elif self.state == "ARM":
            arm=True
            if s.armed:
                self.change("TAKEOFF",now)
            elif elapsed > p.command_timeout:
                self.abort(now,"arming not confirmed")
        else:
            if not s.armed:
                self.change("FAILED",now,"unexpected disarm")
                return Output(np.zeros(3),state=self.state,reason=self.reason)
            desired[2]=np.clip(p.height_kp*(p.height_m-s.height)-p.height_kd*s.pose.v[2],-p.vertical_speed,p.vertical_speed)
            height_ok=abs(s.height-p.height_m) <= p.height_tolerance
            speed_ok=np.linalg.norm(s.pose.v[:2]) <= p.hold_speed and abs(s.pose.v[2]) < .06
            heading_ok=abs(wrap(yaw(s.pose.r)-self.heading)) <= p.yaw_tolerance
            if self.state in ("TAKEOFF","STABILIZE"):
                desired[:2]=limited(p.position_kp*(self.origin[:2]-s.pose.p[:2])-p.velocity_kd*s.pose.v[:2],p.visual_speed)
                if elapsed > p.takeoff_timeout:
                    self.abort(now,"takeoff/stabilization timeout")
                elif height_ok and speed_ok and heading_ok:
                    if self.stable_since is None:
                        self.stable_since=now
                    self.state="STABILIZE"
                    if now-self.stable_since >= p.stable_s:
                        self.change("APPROACH",now)
                else:
                    self.stable_since=None
            elif self.state == "APPROACH":
                delta=s.pose.p-self.origin
                along=float(delta@self.direction)
                cross=delta[:2]-along*self.direction[:2]
                if np.linalg.norm(cross) > p.cross_track_limit:
                    self.abort(now,"cross-track limit exceeded")
                remain=max(0.,p.distance_m-along)
                speed=min(p.forward_speed,math.sqrt(2*p.accel*remain),p.position_kp*remain)
                desired[:2]=limited(speed*self.direction[:2]-p.position_kp*cross-p.velocity_kd*s.pose.v[:2],p.forward_speed)
                obs=s.observation
                if obs and 0 <= now-obs.stamp <= p.panel_timeout and obs.hits >= 3:
                    self.target=obs.panel_id
                    self.anchor=s.pose.p.copy()
                    self.change("BLEND",now)
                elif elapsed > p.approach_timeout or along >= p.distance_m-.10:
                    self.abort(now,"5m reached / search timeout without panel")
            elif self.state in ("BLEND","ALIGN","HOLD"):
                obs=s.observation
                fresh=(obs is not None and obs.panel_id == self.target and
                       0 <= now-obs.stamp <= p.panel_timeout)
                if not fresh:
                    self.hold_since=None
                    if self.lost_since is None:
                        self.lost_since=now
                        self.anchor=s.pose.p.copy()
                    desired[:2]=limited(p.position_kp*(self.anchor[:2]-s.pose.p[:2])-p.velocity_kd*s.pose.v[:2],p.visual_speed)
                    if now-self.lost_since > p.panel_lost_land_s:
                        self.abort(now,"selected panel lost")
                else:
                    self.lost_since=None
                    # The panel relative position comes from the capture-time calibrated ray.
                    # Recompute the camera-centre goal at CURRENT attitude to remove mount offset.
                    error_xy=obs.xyz[:2]-s.pose.p[:2]  # node has shifted xyz to body-centre goal
                    visual=limited(p.position_kp*error_xy-p.velocity_kd*s.pose.v[:2],p.visual_speed)
                    if self.state == "BLEND":
                        alpha=min(1.,elapsed/p.blend_s)
                        brake=limited(p.position_kp*(self.anchor[:2]-s.pose.p[:2])-p.velocity_kd*s.pose.v[:2],p.forward_speed)
                        desired[:2]=(1-alpha)*brake+alpha*visual
                        if alpha >= 1:
                            self.change("ALIGN",now)
                    else:
                        desired[:2]=visual
                        central=(abs(obs.ex) <= p.center_half_width and abs(obs.ey) <= p.center_half_height)
                        if central and height_ok and speed_ok and heading_ok:
                            if self.hold_since is None:
                                self.hold_since=now
                            self.state="HOLD"
                            if now-self.hold_since >= p.hold_s:
                                self.hold_completed=True
                                self.change("LAND",now,"panel centre held 5s")
                        else:
                            self.hold_since=None
                            self.state="ALIGN"
                if now-self.entered > p.align_timeout:
                    self.abort(now,"visual alignment timeout")

        if self.state == "LAND":
            return self.land(now,s)
        dt=max(0.,dt)
        change=desired-self.command
        change[:2]=limited(change[:2],p.accel*dt)
        change[2]=np.clip(change[2],-p.vertical_accel*dt,p.vertical_accel*dt)
        self.command+=change
        yr=float(np.clip(p.yaw_kp*wrap(self.heading-yaw(s.pose.r)),-p.yaw_rate,p.yaw_rate))
        return Output(self.command.copy(),yr,True,mode,arm,self.state,self.reason,self.hold_completed)

    def land(self, now, s):
        self.command[:]=0
        # Never issue AUTO.LAND on an unarmed ground vehicle.
        if not s.armed and s.landed:
            self.success=self.hold_completed
            self.change("DONE" if self.success else "FAILED",now,self.reason)
            return Output(np.zeros(3),state=self.state,reason=self.reason,completed_hold=self.hold_completed)
        if s.mode == "AUTO.LAND":
            if now-self.entered > self.p.land_timeout_s:
                self.change("FAILED",now,"landing completion timeout; FC retains control")
            return Output(np.zeros(3),state=self.state,reason=self.reason,completed_hold=self.hold_completed)
        elapsed=now-self.entered
        # Stop streaming after deadline to permit configured PX4 Offboard-loss handling.
        if elapsed > self.p.land_handover_s:
            self.change("FAILED",now,"AUTO.LAND handover timeout; Offboard-loss handling")
            return Output(np.zeros(3),state=self.state,reason=self.reason,completed_hold=self.hold_completed)
        return Output(np.zeros(3),publish=s.mode=="OFFBOARD",request_mode="AUTO.LAND" if s.armed else "",
                      state=self.state,reason=self.reason,completed_hold=self.hold_completed)
