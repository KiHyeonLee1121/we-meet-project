import json
import math
from pathlib import Path
import yaml
import numpy as np
from .geometry import transform
from .mission import Parameters


def read_yaml(path):
    text=Path(path).read_text().replace("%YAML:1.0","")
    return yaml.safe_load(text)


def load(path):
    config=read_yaml(path)
    Parameters(**config["flight"])
    if config["fc_world_frame"]!="map" or not 0 <= config["panel_height_m"] <= .10:
        raise ValueError("this test requires FC map frame and a flat panel <=0.10m above ground")
    if not .0 <= config["lidar"]["transport_delay_s"] <= .1:
        raise ValueError("LiDAR transport delay outside tested envelope")
    if not config["mavros_namespace"].startswith("/"):
        raise ValueError("absolute MAVROS namespace required")
    return config


def calibration(path):
    c=json.loads(Path(path).read_text())
    required=("verified","resolution","intrinsics","distortion","T_body_imu",
              "T_cam_imu","timeshift_cam_imu","camera_controls","scaler_crop","sensor_clock","imu_topic","imu_frame")
    for name in required:
        if name not in c or c[name] is None:
            raise ValueError("missing measured calibration: "+name)
    if c["verified"] is not True:
        raise ValueError("calibration.verified is false; Pi must verify the actual mounted sensors")
    if not np.isfinite(c["timeshift_cam_imu"]) or abs(c["timeshift_cam_imu"])>.1:
        raise ValueError("camera/IMU time offset outside this synchronised-clock profile")
    if c.get("raw_imu_source_verified") is not True or not c.get("calibration_evidence"):
        raise ValueError("actual IMU stream and calibration evidence required")
    c["T_body_imu"]=transform(c["T_body_imu"],"T_body_imu")
    c["T_cam_imu"]=transform(c["T_cam_imu"],"T_cam_imu")
    c["T_body_camera"]=c["T_body_imu"]@np.linalg.inv(c["T_cam_imu"])
    if c.get("distortion_model") != "radtan":
        raise ValueError("this camera frontend currently requires calibrated pinhole/radtan")
    if len(c["intrinsics"]) != 4 or len(c["distortion"]) != 4 or not np.isfinite(c["intrinsics"]+c["distortion"]).all():
        raise ValueError("invalid intrinsics/distortion")
    fx,fy,cx,cy=c["intrinsics"]
    c["intrinsics"]=list(map(float,c["intrinsics"]))
    c["distortion"]=list(map(float,c["distortion"]))
    w,h=c["resolution"]
    if not (fx>0 and fy>0 and 0<cx<w and 0<cy<h and w<=1280 and h<=960):
        raise ValueError("invalid/test-envelope camera calibration")
    if c["sensor_clock"] not in ("boottime","monotonic") or len(c["scaler_crop"])!=4:
        raise ValueError("verify camera sensor clock and ScalerCrop")
    if c["T_body_camera"][2,2] > -.9:
        raise ValueError("single-panel XY servo requires a downward camera")
    c["lidar_position_body"]=np.asarray(c["lidar_position_body"],dtype=float)
    if c["lidar_position_body"].shape!=(3,) or not np.isfinite(c["lidar_position_body"]).all() or c.get("lidar_geometry_verified") is not True:
        raise ValueError("LiDAR mounting geometry must be measured and verified")
    for name in ("ExposureTime","AnalogueGain","LensPosition","FrameDurationLimits","AeEnable","AfMode"):
        if name not in c["camera_controls"] or c["camera_controls"][name] is None:
            raise ValueError("lock camera capture/calibration control: "+name)
    return c
