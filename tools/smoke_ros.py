#!/usr/bin/env python3
"""Real ROS message/node construction, isolated namespace, hardware disabled.

Run in Jazzy CI or the Pi's ROS container. Uses marked synthetic calibration ONLY here.
"""
import json
import tempfile
from pathlib import Path
import sys
import yaml
import numpy as np
import rclpy
from rclpy.parameter import Parameter

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tests"))
from test_settings import synthetic_calibration
from we_meet_vio.vio_bridge import VioBridge
from we_meet_vio.mission_node import Flight
from we_meet_vio.panel_node import PanelDetector
from we_meet_vio.imu_gate import ImuGate
from we_meet_vio.ros_common import stamp,set_stamp,write_vector,write_quaternion,odom_pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def main():
    with tempfile.TemporaryDirectory() as directory:
        root=Path(directory)
        cfg=yaml.safe_load((ROOT/"config/settings.yaml").read_text())
        cfg["mavros_namespace"]="/isolated_test/mavros"
        cfg["log_directory"]=str(root/"logs")
        (root/"settings.yaml").write_text(yaml.safe_dump(cfg))
        (root/"calibration.json").write_text(json.dumps(synthetic_calibration()))
        rclpy.init(args=["--ros-args","-p","settings_path:="+str(root/"settings.yaml"),
                        "-p","calibration_path:="+str(root/"calibration.json"),"-p","dry_run:=true"])
        nodes=[]
        try:
            bridge=VioBridge()
            nodes.append(bridge)
            flight=Flight()
            nodes.append(flight)
            nodes.append(PanelDetector())
            nodes.append(ImuGate())
            assert bridge.fc_pub is None
            assert flight.command_pub is None and flight.mode_client is None and flight.arm_client is None
            reply=flight.start(Trigger.Request(),Trigger.Response())
            assert not reply.success and "dry_run" in reply.message
            message=Odometry()
            set_stamp(message.header,100.123)
            message.header.frame_id="map"
            message.child_frame_id="base_link"
            write_vector(message.pose.pose.position,[1,2,3])
            write_quaternion(message.pose.pose.orientation,[0,0,0,1])
            write_vector(message.twist.twist.linear,[.1,.2,.3])
            message.pose.covariance=(np.eye(6)*.01).reshape(-1).tolist()
            message.twist.covariance=(np.eye(6)*.01).reshape(-1).tolist()
            pose=odom_pose(message)
            assert abs(stamp(message)-100.123)<1e-9
            np.testing.assert_allclose(pose.v,[.1,.2,.3])
            flight.pose(message)
            assert flight.inputs.pose is not None
            for node in nodes:
                rclpy.spin_once(node,timeout_sec=.01)
            print("Jazzy message adapters and inert node construction passed; no hardware command publishers")
        finally:
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    main()
