"""Host Pi OS -> Jazzy binary mono8 IPC, preserving SensorTimestamp."""
import json
import struct

MAX_PACKET=400000


def pack(meta,pixels):
    header=json.dumps(meta,separators=(",",":")).encode()
    data=struct.pack("!I",len(header))+header+pixels
    if len(header)>4096 or len(data)>MAX_PACKET:
        raise ValueError("camera IPC profile requires <=640x480 mono8")
    return data


def unpack(data):
    if len(data)<4:
        raise ValueError("short camera packet")
    length=struct.unpack("!I",data[:4])[0]
    if length>4096 or length+4>len(data):
        raise ValueError("invalid camera header length")
    meta=json.loads(data[4:4+length])
    pixels=data[4+length:]
    if (meta.get("schema")!=1 or meta.get("encoding")!="mono8" or
        not 0<meta["width"]<=640 or not 0<meta["height"]<=480 or
        len(pixels)!=meta["width"]*meta["height"]):
        raise ValueError("invalid camera dimensions/encoding/payload")
    return meta,pixels


def capture_time(sensor_ns,ros_ns,sensor_now_ns):
    age_ns=sensor_now_ns-int(sensor_ns)
    if not 0 <= age_ns <= 180000000:
        raise ValueError("camera SensorTimestamp clock mismatch/stale")
    return (ros_ns-age_ns)*1e-9
