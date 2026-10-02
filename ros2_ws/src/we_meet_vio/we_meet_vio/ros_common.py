"""Adapters for actual ROS messages; all numeric control is in ROS-free modules."""
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from .geometry import Pose, rotation
from .settings import load, calibration

SENSOR=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
RELIABLE=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE)


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def set_stamp(header, seconds):
    ns=round(seconds*1e9)
    header.stamp.sec=ns//1000000000
    header.stamp.nanosec=ns%1000000000


def vector(v):
    return np.array([v.x,v.y,v.z])


def quaternion(q):
    return np.array([q.x,q.y,q.z,q.w])


def write_vector(message,values):
    message.x,message.y,message.z=map(float,values)


def write_quaternion(message,values):
    message.x,message.y,message.z,message.w=map(float,values)


def odom_pose(msg):
    q=quaternion(msg.pose.pose.orientation)
    v=rotation(q)@vector(msg.twist.twist.linear)
    return Pose(stamp(msg),vector(msg.pose.pose.position),q,v,vector(msg.twist.twist.angular),
                np.array(msg.pose.covariance).reshape(6,6),np.array(msg.twist.covariance).reshape(6,6))


def setup(node):
    node.declare_parameter("settings_path","")
    node.declare_parameter("calibration_path","")
    node.declare_parameter("dry_run",True)
    node.settings=load(node.get_parameter("settings_path").value)
    node.cal=calibration(node.get_parameter("calibration_path").value)
    node.dry=bool(node.get_parameter("dry_run").value)
    return node.settings,node.cal


def run(factory):
    rclpy.init()
    node=None
    try:
        node=factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
