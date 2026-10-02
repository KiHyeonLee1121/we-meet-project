"""Actual OpenVINS -> body FLU/ROS map -> MAVROS odometry adapter."""
import json
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from mavros_msgs.msg import State, TimesyncStatus
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from .geometry import Pose, Alignment, covariance,rotation
from .health import StreamGuard, ClockGuard
from .ros_common import (SENSOR,RELIABLE,stamp,set_stamp,vector,quaternion,write_vector,
                         write_quaternion,odom_pose,setup,run)


class VioBridge(Node):
    def __init__(self):
        super().__init__("vio_bridge")
        cfg,cal=setup(self)
        self.cfg=cfg
        self.ns=cfg["mavros_namespace"].rstrip("/")
        self.align=Alignment(cal["T_body_imu"])
        self.clock=ClockGuard()
        self.stream_guards={"imu":StreamGuard(180,.08,.025),
                     "camera":StreamGuard(18,.18,.10),
                     "camera_update":StreamGuard(12,.25,.12),
                     "vio":StreamGuard(25,.15,.08)}
        self.fc=None
        self.fc_stamp=0.
        self.armed=False
        self.state_stamp=0.
        self.sync_stamp=0.
        self.rtt=1e6
        self.failed=""
        self.last_imu=None
        self.last_source=None
        self.last_pub=0.
        self.latest=None
        self.update_stamp=0.
        self.image_stamp=0.
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.frames_ok=False
        self.vision_owner_ok=False
        self.vision_subscriber_ok=False
        self.body_pub=self.create_publisher(Odometry,"/vio/body_odometry",RELIABLE)
        self.fc_pub=None if self.dry else self.create_publisher(Odometry,self.ns+"/odometry/out",RELIABLE)
        self.status=self.create_publisher(String,"/vio/status",RELIABLE)
        self.create_subscription(Imu,"/vio/imu",self.imu,SENSOR)
        self.create_subscription(Image,"/camera/image_raw",self.image,SENSOR)
        self.create_subscription(PoseWithCovarianceStamped,"/ov_msckf/poseimu",self.update,SENSOR)
        self.create_subscription(Odometry,"/ov_msckf/odomimu",self.odometry,SENSOR)
        self.create_subscription(PoseStamped,self.ns+"/local_position/pose",self.fc_pose,SENSOR)
        self.create_subscription(State,self.ns+"/state",self.state,SENSOR)
        self.create_subscription(TimesyncStatus,self.ns+"/timesync_status",self.sync,SENSOR)
        self.create_timer(.05,self.report)

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def state(self,msg):
        self.armed=msg.armed
        self.state_stamp=self.now_s()

    def fc_pose(self,msg):
        try:
            q=quaternion(msg.pose.orientation)
            p=vector(msg.pose.position)
            if msg.header.frame_id != self.cfg["fc_world_frame"] or not np.isfinite(p).all():
                raise ValueError("FC pose frame/position mismatch")
            self.fc=Pose(stamp(msg),p,q,np.zeros(3),np.zeros(3),np.eye(6),np.eye(6))
            self.fc.r
            self.fc_stamp=stamp(msg)
        except ValueError as e:
            self.fc=None
            self.fc_stamp=0.
            if self.armed or self.align.a is not None:
                self.failed=str(e)

    def sync(self,msg):
        self.sync_stamp=self.now_s()
        self.rtt=msg.round_trip_time_ms

    def imu(self,msg):
        now=self.now_s()
        if msg.header.frame_id != self.cal["imu_frame"]:
            self.failed="IMU frame mismatch; recalibrate/remap actual vectors, not only frame_id"
            return
        if not np.isfinite(np.r_[vector(msg.angular_velocity),vector(msg.linear_acceleration)]).all():
            self.failed="invalid IMU"
            return
        if not self.armed and not 8 < np.linalg.norm(vector(msg.linear_acceleration)) < 12:
            self.stream_guards["imu"].error="stationary acceleration not ~9.81m/s2; raw counts/gravity removed?"
            return
        self.stream_guards["imu"].feed(stamp(msg),now)

    def image(self,msg):
        if msg.header.frame_id != "camera_optical" or [msg.width,msg.height] != self.cal["resolution"]:
            self.failed="capture frame/resolution differs from calibration"
            return
        if self.stream_guards["camera"].feed(stamp(msg),self.now_s()):
            self.image_stamp=stamp(msg)

    def update(self,msg):
        # poseimu uses IMU-clock corrected camera time, distinct from propagated odomimu.
        if self.stream_guards["camera_update"].feed(stamp(msg),self.now_s()):
            self.update_stamp=stamp(msg)

    def odometry(self,msg):
        now=self.now_s()
        try:
            if msg.header.frame_id != "global" or msg.child_frame_id != "imu":
                raise ValueError("unexpected pinned OpenVINS output frame")
            if not self.clock.check(now,time.monotonic()):
                raise ValueError("ROS clock stepped; restart all streams disarmed")
            source=odom_pose(msg)
            if not np.isfinite(np.r_[source.p,source.v,source.omega]).all():
                raise ValueError("nonfinite OpenVINS state; not forwarded to FC")
            covariance(source.pose_cov)
            covariance(source.twist_cov)
            accepted=self.stream_guards["vio"].feed(source.stamp,now)
            if self.last_source and source.stamp <= self.last_source.stamp:
                raise ValueError("OpenVINS restart/time reversal; alignment must not silently relatch")
            if self.last_source:
                dt=source.stamp-self.last_source.stamp
                if np.linalg.norm(source.p-self.last_source.p-self.last_source.v*dt) > .25:
                    raise ValueError("OpenVINS position discontinuity")
            self.last_source=source
            if not accepted:
                return
            if self.align.a is None:
                if self.armed or self.fc is None or not 0 <= now-self.fc_stamp < .2 or now-self.state_stamp > 1.:
                    return
                self.align.latch(source,self.fc)
            body=self.align.apply(source)
            self.latest=body
            # HIGHRES_IMU ~200Hz input; limit FC external odometry to 40Hz.
            if source.stamp-self.last_pub < .024:
                return
            self.last_pub=source.stamp
            out=Odometry()
            set_stamp(out.header,source.stamp)  # do not replace capture/measurement time with now
            out.header.frame_id="map"
            out.child_frame_id="base_link"
            write_vector(out.pose.pose.position,body.p)
            write_quaternion(out.pose.pose.orientation,body.q)
            write_vector(out.twist.twist.linear,body.r.T@body.v)
            write_vector(out.twist.twist.angular,body.omega)
            out.pose.covariance=body.pose_cov.reshape(-1).tolist()
            out.twist.covariance=body.twist_cov.reshape(-1).tolist()
            self.body_pub.publish(out)
            # Bridge may feed FC before mission start to let EKF converge, but NEVER
            # before current camera corrections/synchronisation are valid.
            if self.fc_pub and not self.failed and self.sensor_ready(now):
                self.fc_pub.publish(out)
        except (ValueError,TypeError) as e:
            self.failed=str(e)

    def sensor_ready(self,now):
        return (not self.failed and self.align.a is not None and
                all(g.ready(now) for g in self.stream_guards.values()) and
                self.frames_ok and self.vision_owner_ok and self.vision_subscriber_ok and now-self.sync_stamp < 1.5 and self.rtt < 10 and not self.clock.failed)

    def report(self):
        now=self.now_s()
        self.clock.check(now,time.monotonic()) # pair clocks before TF/graph lookups take time
        permitted=1 if self.fc_pub else 0
        self.vision_subscriber_ok=self.count_subscribers(self.ns+"/odometry/out")>=1
        self.vision_owner_ok=(self.count_publishers(self.ns+"/odometry/out")<=permitted and
                              self.count_publishers(self.ns+"/vision_pose/pose")==0 and
                              self.count_publishers(self.ns+"/vision_pose/pose_cov")==0)
        # MAVROS 2.14 odometry plugin relies on these static rotations.
        # Never add another ENU/NED conversion to the payload.
        self.frames_ok=False
        if (self.tf_buffer.can_transform("map_ned","map",Time()) and
            self.tf_buffer.can_transform("base_link_frd","base_link",Time())):
            try:
                world=self.tf_buffer.lookup_transform("map_ned","map",Time()).transform
                body=self.tf_buffer.lookup_transform("base_link_frd","base_link",Time()).transform
                self.frames_ok=(np.allclose(rotation(quaternion(world.rotation)),[[0,1,0],[1,0,0],[0,0,-1]],atol=1e-5) and
                                np.allclose(rotation(quaternion(body.rotation)),np.diag([1.,-1,-1]),atol=1e-5) and
                                np.linalg.norm(vector(world.translation))+np.linalg.norm(vector(body.translation))<1e-5)
            except (ValueError,RuntimeError,TransformException):
                self.frames_ok=False
        ready=self.sensor_ready(now)
        reasons=[self.failed] if self.failed else []
        if self.align.a is None:
            reasons.append("waiting for disarmed, fresh valid FC pose to latch alignment")
        reasons += [name+": "+(g.error or "rate/window/staleness") for name,g in self.stream_guards.items() if not g.ready(now)]
        if now-self.sync_stamp >= 1.5 or self.rtt >= 10:
            reasons.append("MAVROS time synchronisation missing/high RTT")
        if not self.frames_ok:
            reasons.append("MAVROS map_ned/base_link_frd TF missing")
        if not self.vision_owner_ok:
            reasons.append("other external vision publisher")
        if not self.vision_subscriber_ok:
            reasons.append("MAVROS odometry plugin has no input subscriber")
        msg=String()
        msg.data=json.dumps({"stamp":now,"ready":ready,"reason":"; ".join(reasons),
                             "image_stamp":self.image_stamp,"update_stamp":self.update_stamp,
                             "rates":{k:g.rate() for k,g in self.stream_guards.items()},"rtt_ms":self.rtt,
                             "aligned":self.align.a is not None,"dry_run":self.dry})
        self.status.publish(msg)


def main():
    run(VioBridge)
