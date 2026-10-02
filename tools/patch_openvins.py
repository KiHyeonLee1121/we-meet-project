#!/usr/bin/env python3
"""Small reproducible patches to the pinned upstream checkout; estimator maths unchanged."""
import argparse
import hashlib
import json
from pathlib import Path


def patch(root):
    root=Path(root)
    hashes={}
    for rel in ("ov_msckf/src/ros/ROS2Visualizer.h","ov_msckf/src/ros/ROSVisualizerHelper.h"):
        file=root/rel
        text=file.read_text()
        for header in ("image_transport/image_transport","tf2_geometry_msgs/tf2_geometry_msgs","cv_bridge/cv_bridge"):
            old="#include <"+header+".h>"
            new="#if __has_include(<"+header+".hpp>)\n#include <"+header+".hpp>\n#else\n"+old+"\n#endif"
            if old in text and new not in text:
                text=text.replace(old,new)
        file.write_text(text)
        hashes[rel]=hashlib.sha256(text.encode()).hexdigest()
    for package in ("ov_core","ov_init","ov_msckf"):
        cmake=root/(package+"/CMakeLists.txt")
        text=cmake.read_text().replace("set(CMAKE_CXX_STANDARD 14)","set(CMAKE_CXX_STANDARD 17)")
        cmake.write_text(text)
        hashes[package+"/CMakeLists.txt"]=hashlib.sha256(text.encode()).hexdigest()
        rel=package+"/cmake/ROS2.cmake"
        file=root/rel
        text=file.read_text()
        if "${EIGEN3_INCLUDE_DIRS}" not in text:
            text=text.replace("        ${EIGEN3_INCLUDE_DIR}","        ${EIGEN3_INCLUDE_DIRS}\n        ${EIGEN3_INCLUDE_DIR}")
        file.write_text(text)
        hashes[rel]=hashlib.sha256(text.encode()).hexdigest()
    rel="ov_msckf/src/ros/ROS2Visualizer.cpp"
    file=root/rel
    text=file.read_text()
    old="cam_topic, 10, [this, i](const sensor_msgs::msg::Image::SharedPtr msg0)"
    new="cam_topic, rclcpp::SensorDataQoS().keep_last(2), [this, i](const sensor_msgs::msg::Image::SharedPtr msg0)"
    if old not in text and new not in text:
        raise ValueError("pinned monocular subscription not found")
    text=text.replace(old,new)
    old="  std::sort(camera_queue.begin(), camera_queue.end());"
    new=old+"\n  // WE_MEET: bounded pending images; mono experiment, prefer fresh data.\n  while (camera_queue.size() > 2) camera_queue.pop_front();"
    if new not in text:
        if text.count(old)!=2:
            raise ValueError("unexpected pinned camera queue layout")
        text=text.replace(old,new)
    file.write_text(text)
    hashes[rel]=hashlib.sha256(text.encode()).hexdigest()
    (root/"WE_MEET_PATCH.json").write_text(json.dumps(hashes,indent=2))
    return hashes


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("checkout")
    args=parser.parse_args()
    print(json.dumps(patch(args.checkout),indent=2))
