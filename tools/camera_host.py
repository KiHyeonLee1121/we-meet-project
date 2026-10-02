#!/usr/bin/env python3
"""Pi OS host capture; no ROS, FC connection, image codec or remote GPU required."""
import argparse
import json
import socket
import time
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2_ws/src/we_meet_vio"))
from we_meet_vio.camera_wire import pack


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--profile")
    parser.add_argument("--socket",default="/run/we-meet-vio/camera.sock")
    parser.add_argument("--probe",action="store_true")
    parser.add_argument("--lens-position",type=float,default=.33,help="probe-only initial focus setting")
    args=parser.parse_args()
    from picamera2 import Picamera2
    from libcamera import controls
    if not args.probe and not args.profile:
        parser.error("--profile is required")
    profile=json.loads(Path(args.profile).read_text()) if args.profile else {
        "resolution":[640,480],"camera_controls":{"ExposureTime":4000,"AnalogueGain":2.,
        "LensPosition":args.lens_position,"FrameDurationLimits":[40000,40000],"AeEnable":False,"AfMode":"Manual"},
        "scaler_crop":None,"sensor_clock":"boottime"}
    if profile["resolution"]!=[640,480]:
        raise ValueError("initial IPC profile supports calibrated 640x480 only")
    options=dict(profile["camera_controls"])
    if options["AfMode"]!="Manual" or options["AeEnable"] is not False or options["LensPosition"] is None:
        raise ValueError("lock manual focus/exposure before capture")
    options["AfMode"]=controls.AfModeEnum.Manual
    options["FrameDurationLimits"]=tuple(options["FrameDurationLimits"])
    if profile["scaler_crop"] is not None:
        options["ScalerCrop"]=tuple(profile["scaler_crop"])
    camera=Picamera2()
    camera.configure(camera.create_video_configuration(main={"size":(640,480),"format":"YUV420"},buffer_count=4,queue=False))
    camera.set_controls(options)
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,2*1024*1024)
    sock.setblocking(False)
    camera.start()
    frames=drops=0
    report=time.monotonic()
    try:
        while True:
            request=camera.capture_request()
            try:
                actual=request.get_metadata()
                pixels=np.ascontiguousarray(request.make_array("main")[:480,:640]).tobytes()
                meta={"schema":1,"encoding":"mono8","width":640,"height":480,
                      "sensor_ns":int(actual["SensorTimestamp"]),"sensor_clock":profile["sensor_clock"],
                      "ExposureTime":int(actual["ExposureTime"]),"LensPosition":float(actual["LensPosition"]),
                      "ScalerCrop":list(actual["ScalerCrop"])}
                frames+=1
                if args.probe:
                    if frames>=10:
                        print(json.dumps(meta,indent=2))
                        break
                    continue
                if profile["scaler_crop"] is None:
                    raise ValueError("fill actual ScalerCrop from --probe")
                try:
                    sock.sendto(pack(meta,pixels),args.socket)
                except (FileNotFoundError,ConnectionRefusedError,BlockingIOError):
                    drops+=1
                if time.monotonic()-report>=5:
                    print(json.dumps({"captured":frames,"ipc_dropped":drops,"socket":args.socket}),flush=True)
                    report=time.monotonic()
            finally:
                request.release()
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        camera.close()
        sock.close()


if __name__=="__main__":
    main()
