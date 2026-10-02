import math
import serial
from rclpy.node import Node
from sensor_msgs.msg import Range
from .lidar import Parser
from .ros_common import setup,run,SENSOR,set_stamp


class Lidar(Node):
    def __init__(self):
        super().__init__("tf_luna")
        cfg,_=setup(self)
        self.cfg=cfg["lidar"]
        self.parser=Parser()
        self.serial=serial.Serial(self.cfg["port"],self.cfg["baud"],timeout=0,exclusive=True)
        self.serial.reset_input_buffer()
        self.pub=self.create_publisher(Range,"/lidar/range",SENSOR)
        self.create_timer(.005,self.poll)

    def poll(self):
        count=self.serial.in_waiting
        if count>90:
            self.serial.reset_input_buffer()
            self.parser=Parser()
            self.get_logger().error("LiDAR serial backlog: discard, do not stamp old samples as new",throttle_duration_sec=1.)
            return
        if not count:
            return
        frames=self.parser.feed(self.serial.read(count))
        if not frames:
            return
        f=frames[-1]  # only latest, no duplicated timestamps for serial batch
        if not .2<=f.distance<=8 or not self.cfg["minimum_strength"]<=f.strength<65535:
            return
        msg=Range()
        set_stamp(msg.header,self.get_clock().now().nanoseconds*1e-9-self.cfg["transport_delay_s"])
        msg.header.frame_id="lidar_down"
        msg.radiation_type=Range.INFRARED
        msg.field_of_view=.035
        msg.min_range,msg.max_range=.2,8.
        msg.range=f.distance
        self.pub.publish(msg)

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


def main():
    run(Lidar)
