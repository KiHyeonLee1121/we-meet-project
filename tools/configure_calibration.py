#!/usr/bin/env python3
"""Convert ACTUAL Kalibr camera/IMU + mounted rig measurements to shared config.

No calibration values are estimated/invented here; nulls and a mismatched camera are rejected.
"""
import argparse
import json
from pathlib import Path
import sys
import yaml
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2_ws/src/we_meet_vio"))
from we_meet_vio.settings import read_yaml,calibration
from we_meet_vio.geometry import transform


class OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self,flow=False,indentless=False):
        return super().increase_indent(flow,False)


def flow_sequence(dumper,sequence):
    return dumper.represent_sequence("tag:yaml.org,2002:seq",sequence,flow_style=True)


OpenCVDumper.add_representer(list,flow_sequence)


def opencv_yaml(data):
    return "%YAML:1.0\n"+yaml.dump(data,Dumper=OpenCVDumper,sort_keys=False)


def generate(camera_file,imu_file,mount_file,out):
    camera=read_yaml(camera_file)["cam0"]
    imu=read_yaml(imu_file)["imu0"]
    mount=read_yaml(mount_file)
    for name in ("intrinsics","distortion_coeffs","resolution","T_cam_imu","timeshift_cam_imu"):
        if name not in camera or camera[name] is None:
            raise ValueError("missing Kalibr camera field: "+name)
    if camera["camera_model"]!="pinhole" or camera["distortion_model"]!="radtan":
        raise ValueError("this first module supports pinhole/radtan only")
    transform(camera["T_cam_imu"],"T_cam_imu")
    transform(mount["T_body_imu"],"T_body_imu")
    for key in ("accelerometer_noise_density","accelerometer_random_walk","gyroscope_noise_density","gyroscope_random_walk","update_rate"):
        if key not in imu or not np.isfinite(imu[key]) or imu[key] <= 0:
            raise ValueError("missing measured IMU noise/rate: "+key)
    if imu["update_rate"]<180:
        raise ValueError("IMU source below this profile's 180Hz readiness limit")
    if mount.get("raw_imu_source_verified") is not True:
        raise ValueError("verify source stream, units, FLU frame, clock and inclusion of gravity first")
    if "model" not in imu:
        if mount.get("imu_intrinsics_already_corrected") is not True:
            raise ValueError("IMU intrinsic model missing; verify already-corrected source or supply actual matrices")
        imu["model"]="calibrated"
    for name in ("Tw","Ta","R_IMUtoACC","R_IMUtoGYRO","Tg"):
        if name not in imu:
            if imu["model"]!="calibrated" or mount.get("imu_intrinsics_already_corrected") is not True:
                raise ValueError("missing actual IMU intrinsic field: "+name)
            # Identity means no ADDITIONAL correction to an already calibrated SI stream,
            # not that the physical sensor's unknown intrinsic errors were measured as zero.
            imu[name]=(np.zeros((3,3)) if name=="Tg" else np.eye(3)).tolist()
    camera["rostopic"]="/camera/image_raw"
    imu["rostopic"]="/vio/imu"
    cal=dict(mount)
    cal.update({"resolution":camera["resolution"],"intrinsics":camera["intrinsics"],
                "distortion":camera["distortion_coeffs"],"distortion_model":camera["distortion_model"],
                "T_cam_imu":camera["T_cam_imu"],"timeshift_cam_imu":camera["timeshift_cam_imu"]})
    out=Path(out)
    out.mkdir(parents=True,exist_ok=True)
    if (out/"calibration.json").exists() or (out/"openvins/kalibr_imucam_chain.yaml").exists():
        raise ValueError("output calibration already exists; use a new directory to review changes")
    serialized=json.dumps(cal,indent=2)
    # Validate the exact runtime calibration BEFORE writing any deployable files.
    import tempfile
    with tempfile.TemporaryDirectory() as temporary:
        candidate=Path(temporary)/"calibration.json"
        candidate.write_text(serialized)
        calibration(candidate)
    ov=out/"openvins"
    ov.mkdir(exist_ok=True)
    (out/"calibration.json").write_text(serialized)
    (ov/"kalibr_imucam_chain.yaml").write_text(opencv_yaml({"cam0":camera}))
    (ov/"kalibr_imu_chain.yaml").write_text(opencv_yaml({"imu0":imu}))
    (ov/"estimator_config.yaml").write_text((ROOT/"config/openvins/estimator_config.yaml").read_text())
    for name in ("settings.yaml","mavros_overrides.yaml"):
        if not (out/name).exists():
            (out/name).write_text((ROOT/"config"/name).read_text())
    return cal


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--camera",required=True)
    parser.add_argument("--imu",required=True)
    parser.add_argument("--mount",required=True)
    parser.add_argument("--out",required=True)
    args=parser.parse_args()
    generate(args.camera,args.imu,args.mount,args.out)
    print("wrote measured calibration; flight/fusion verification remains false in settings.yaml")
