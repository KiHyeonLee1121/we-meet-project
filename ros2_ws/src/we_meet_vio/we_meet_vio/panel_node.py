import json
import time
from threading import Lock
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .ros_common import setup,run,SENSOR,RELIABLE,stamp
from .perception import detect


class PanelDetector(Node):
    def __init__(self):
        super().__init__("panel_detector")
        cfg,_=setup(self)
        cv2.setNumThreads(1)
        self.cfg=cfg["detector"]
        self.image=None
        self.lock=Lock()
        self.pub=self.create_publisher(String,"/panels/detections",RELIABLE)
        self.create_subscription(Image,"/camera/image_raw",self.receive,SENSOR)
        self.create_timer(1./self.cfg["frequency_hz"],self.process)

    def receive(self,msg):
        with self.lock:
            self.image=msg  # overwrite; no queue of stale inference frames

    def process(self):
        with self.lock:
            msg,self.image=self.image,None
        if msg is None:
            return
        now=self.get_clock().now().nanoseconds*1e-9
        if not 0 <= now-stamp(msg) <= .20 or msg.encoding != "mono8":
            return
        started=time.monotonic()
        gray=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
        try:
            detections=detect(gray,self.cfg["dark_threshold"],self.cfg["min_area_fraction"],self.cfg["max_area_fraction"])
            features=cv2.goodFeaturesToTrack(gray,maxCorners=120,qualityLevel=.01,minDistance=12)
            points=[] if features is None else features.reshape(-1,2)
            cells={(int(p[0]*3/msg.width),int(p[1]*3/msg.height)) for p in points}
            out=String()
            out.data=json.dumps({"schema":1,"stamp":stamp(msg),"frame_id":msg.header.frame_id,
                                 "width":msg.width,"height":msg.height,"panels":detections,
                                 "texture_points":len(points),"texture_cells":len(cells),
                                 "processing_ms":1000*(time.monotonic()-started)})
            self.pub.publish(out)
        except (ValueError,cv2.error) as e:
            self.get_logger().error(str(e),throttle_duration_sec=2.)


def main():
    run(PanelDetector)
