#!/usr/bin/env python3
"""Read-only ROS stream rate/capture-age report; no FC publishers or command services."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,Imu,Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--seconds",type=float,default=30)
    parser.add_argument("--imu-topic",default="/mavros/imu/data_raw")
    parser.add_argument("--out",required=True)
    args=parser.parse_args()
    if not 5<=args.seconds<=120:
        parser.error("seconds must be 5..120")
    rclpy.init()
    node=Node("vio_readonly_benchmark")
    streams={}
    for topic,msgtype in (("/camera/image_raw",Image),(args.imu_topic,Imu),("/vio/imu",Imu),
                          ("/ov_msckf/poseimu",PoseWithCovarianceStamped),
                          ("/vio/body_odometry",Odometry),("/lidar/range",Range)):
        streams[topic]=[]
        def callback(msg,key=topic):
            source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=node.get_clock().now().nanoseconds*1e-9
            streams[key].append((source,(now-source)*1000))
        node.create_subscription(msgtype,topic,callback,qos_profile_sensor_data)
    deadline=time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report={"duration_s":args.seconds,"read_only":True,"streams":{}}
    for topic,data in streams.items():
        if len(data)<2:
            report["streams"][topic]={"samples":len(data),"ready":False}
            continue
        array=np.array(data)
        dt=np.diff(array[:,0])
        report["streams"][topic]={"samples":len(data),"source_hz":float(1/np.mean(dt)) if np.mean(dt)>0 else 0,
            "max_gap_ms":float(np.max(dt)*1000),"repeated_or_reverse_stamps":int(np.sum(dt<=0)),
            "capture_age_ms_p50_p95_p99":np.percentile(array[:,1],[50,95,99]).tolist()}
    Path(args.out).write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
