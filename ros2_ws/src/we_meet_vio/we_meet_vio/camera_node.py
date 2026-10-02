"""Receive latest local host frame; raw calibration collection is a separate mode."""
import time
import json
import os
import socket
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from .ros_common import setup,run,SENSOR,set_stamp
from .camera_wire import MAX_PACKET,unpack,capture_time


class PiCamera(Node):
    def __init__(self):
        super().__init__("pi_camera")
        self.declare_parameter("socket_path","/run/we-meet-vio/camera.sock")
        self.declare_parameter("calibration_only",False)
        self.declare_parameter("capture_profile","")
        calibration_only=self.get_parameter("calibration_only").value
        if calibration_only:
            self.cal=json.loads(Path(self.get_parameter("capture_profile").value).read_text())
        else:
            setup(self)
        cal=self.cal
        if cal["resolution"]!=[640,480] or cal["scaler_crop"] is None or cal["camera_controls"]["LensPosition"] is None:
            raise ValueError("fill chosen 640x480 capture profile using host --probe metadata")
        if cal["camera_controls"]["AeEnable"] is not False or cal["camera_controls"]["AfMode"]!="Manual":
            raise ValueError("focus/exposure must be locked for VIO calibration")
        if cal["sensor_clock"] not in ("boottime","monotonic"):
            raise ValueError("unsupported sensor clock")
        self.clock_id=time.CLOCK_BOOTTIME if cal["sensor_clock"]=="boottime" else time.CLOCK_MONOTONIC
        self.last_stamp=0.
        self.image_pub=self.create_publisher(Image,"/camera/image_raw",SENSOR)
        self.info_pub=None if calibration_only else self.create_publisher(CameraInfo,"/camera/camera_info",SENSOR)
        self.socket_path=self.get_parameter("socket_path").value
        Path(self.socket_path).parent.mkdir(parents=True,exist_ok=True)
        self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,2*1024*1024)
        self.sock.bind(self.socket_path) # refuse existing owner, do not unlink someone else's socket
        os.chmod(self.socket_path,0o660) # shared setgid directory permits the host capture group
        self.sock.setblocking(False)
        self.create_timer(.005,self.receive)

    def receive(self):
        latest=None
        for _ in range(32):
            try:
                latest=self.sock.recv(MAX_PACKET)
            except BlockingIOError:
                break
        if latest is None:
            return
        try:
            meta,pixels=unpack(latest)
            if [meta["width"],meta["height"]]!=self.cal["resolution"] or meta["sensor_clock"]!=self.cal["sensor_clock"]:
                raise ValueError("capture profile/clock mismatch")
            if meta["ScalerCrop"]!=self.cal["scaler_crop"]:
                raise ValueError("capture ScalerCrop differs from calibration")
            if abs(meta["LensPosition"]-self.cal["camera_controls"]["LensPosition"])>.02:
                raise ValueError("capture focus differs from calibration")
            if abs(meta["ExposureTime"]-self.cal["camera_controls"]["ExposureTime"])>300:
                raise ValueError("capture exposure differs from calibration")
            ros_ns=self.get_clock().now().nanoseconds
            sensor_now_ns=time.clock_gettime_ns(self.clock_id)
            source=capture_time(meta["sensor_ns"],ros_ns,sensor_now_ns)
            if source<=self.last_stamp:
                raise ValueError("capture clock reversal")
            self.last_stamp=source
            self.publish(source,pixels)
        except (ValueError,KeyError,TypeError) as e:
            self.get_logger().error(str(e),throttle_duration_sec=2.)

    def publish(self,source,pixels):
        w,h=self.cal["resolution"]
        msg=Image()
        set_stamp(msg.header,source)
        msg.header.frame_id="camera_optical"
        msg.height,msg.width=h,w
        msg.encoding="mono8"
        msg.is_bigendian=0
        msg.step=w
        msg.data=pixels
        self.image_pub.publish(msg)
        if self.info_pub is None:
            return # no invented intrinsics or FC publisher in calibration collection
        info=CameraInfo()
        info.header=msg.header
        info.height,info.width=h,w
        info.distortion_model="plumb_bob"
        info.d=list(self.cal["distortion"])
        fx,fy,cx,cy=self.cal["intrinsics"]
        info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.]
        info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
        self.info_pub.publish(info)

    def destroy_node(self):
        self.sock.close()
        Path(self.socket_path).unlink(missing_ok=True)
        return super().destroy_node()


def main():
    run(PiCamera)
