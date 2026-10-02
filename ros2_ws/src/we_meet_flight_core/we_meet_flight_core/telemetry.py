"""Read-only PX4 MAVLink ODOMETRY decoding; no serial port or FC writes.

Wire offsets follow mavlink/common/mavlink_msg_odometry.h (message 331).
PX4's reset counter is the uint8 sum of attitude/position/velocity resets,
not a yaw-only counter. Treat any change during a trial as unsafe.
"""
from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class Odometry:
    timestamp_us: int
    reset_counter: int
    position_enu: tuple
    velocity_enu: tuple
    position_sigma_m: float
    velocity_sigma_mps: float


def horizontal_sigma(xx, xy, yy):
    """Largest horizontal covariance eigenvalue, not just the X diagonal."""
    if not all(math.isfinite(v) for v in (xx, xy, yy)) or xx < 0 or yy < 0 or xy*xy > xx*yy+1e-12:
        return math.inf
    return math.sqrt(max(0.0, (xx+yy+math.hypot(xx-yy, 2*xy))/2))


def decode_odometry(msg, system_id, component_id):
    if (msg.msgid != 331 or msg.framing_status != 1 or msg.magic != 253
            or msg.sysid != system_id or msg.compid != component_id):
        return None
    length = int(msg.len)
    if not 232 <= length <= 233 or len(msg.payload64)*8 < length:
        return None  # Autopilot estimator_type extension must be present.
    try:
        payload = b''.join(struct.pack('<Q', int(word)) for word in msg.payload64)[:length].ljust(233, b'\0')
        if payload[228] != 1 or payload[229] != 1 or payload[231] != 8:
            return None  # Only LOCAL_NED pose + velocity from the FC estimator.
        if struct.unpack_from('<b', payload, 232)[0] < 0:
            return None
        timestamp = struct.unpack_from('<Q', payload)[0]
        p = struct.unpack_from('<3f', payload, 8)
        v = struct.unpack_from('<3f', payload, 36)
        pc = struct.unpack_from('<21f', payload, 60)
        vc = struct.unpack_from('<21f', payload, 144)
        if timestamp == 0 or not all(math.isfinite(x) for x in (*p, *v)):
            return None
        # PX4 emits NaN off-diagonals but valid diagonal variances.
        # These sigma values are marginal-axis estimates, not ground truth.
        ps = max(math.sqrt(x) for x in (pc[0], pc[6])) if all(math.isfinite(x) and x > 0 for x in (pc[0], pc[6])) else math.inf
        vs = max(math.sqrt(x) for x in (vc[0], vc[6])) if all(math.isfinite(x) and x > 0 for x in (vc[0], vc[6])) else math.inf
        return Odometry(timestamp, payload[230], (p[1], p[0], -p[2]), (v[1], v[0], -v[2]), ps, vs)
    except (ValueError, OverflowError, struct.error):
        return None
