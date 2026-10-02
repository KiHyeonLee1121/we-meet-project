"""Reject unsynchronised/duplicated/wrong-unit IMU before OpenVINS initialisation."""
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import TimesyncStatus
from std_msgs.msg import String
from .health import StreamGuard,ClockGuard
from .ros_common import setup,run,SENSOR,RELIABLE,stamp,vector


class ImuGate(Node):
    def __init__(self):
        super().__init__("vio_imu_gate")
        cfg,cal=setup(self)
        self.guard=StreamGuard(180,.08,.025)
        self.clock=ClockGuard()
        self.sync_stamp=0.
        self.rtt=1e6
        self.streaming=False
        self.failed=""
        self.pub=self.create_publisher(Imu,"/vio/imu",SENSOR)
        self.status=self.create_publisher(String,"/vio/imu_gate_status",RELIABLE)
        self.create_subscription(Imu,cal["imu_topic"],self.receive,SENSOR)
        self.create_subscription(TimesyncStatus,cfg["mavros_namespace"].rstrip("/")+"/timesync_status",self.sync,SENSOR)

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def sync(self,msg):
        self.sync_stamp=self.now_s()
        self.rtt=msg.round_trip_time_ms

    def receive(self,msg):
        now=self.now_s()
        valid=(msg.header.frame_id==self.cal["imu_frame"] and
               np.isfinite(np.r_[vector(msg.angular_velocity),vector(msg.linear_acceleration)]).all() and
               self.clock.check(now,time.monotonic()) and
               now-self.sync_stamp<1.5 and np.isfinite(self.rtt) and self.rtt<10)
        if not self.streaming:
            valid=valid and 8<np.linalg.norm(vector(msg.linear_acceleration))<12
        valid=bool(valid and self.guard.feed(stamp(msg),now))
        if self.streaming and not valid:
            self.failed="IMU source/time invalid after streaming; restart VIO pipeline disarmed"
        if self.failed:
            return
        if valid and self.guard.ready(now):
            self.streaming=True
            self.pub.publish(msg) # unchanged stamp/frame/values; no interpolation
        elif int(now*10)%10==0:
            status=String()
            status.data="waiting for synchronised >=180Hz calibrated SI/gravity-included IMU"
            self.status.publish(status)


def main():
    run(ImuGate)
