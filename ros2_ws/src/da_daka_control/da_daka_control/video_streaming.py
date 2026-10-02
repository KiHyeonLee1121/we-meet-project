"""Pure command construction for Raspberry Pi camera streaming."""

import ipaddress


def build_rpicam_command(
    *,
    executable: str,
    laptop_ip: str,
    port: int,
    width: int,
    height: int,
    framerate: int,
    bitrate_bps: int,
    shutter_us: int = 0,
    gain: float = 0.0,
) -> list[str]:
    """Build an argument-only MPEG-TS command without invoking a shell."""
    if not executable:
        raise ValueError('rpicam executable cannot be empty')
    address = ipaddress.ip_address(laptop_ip)
    if not 1 <= port <= 65535:
        raise ValueError('port must be within [1, 65535]')
    if min(width, height, framerate, bitrate_bps) <= 0:
        raise ValueError('video dimensions/rate/bitrate must be positive')
    manual_exposure = shutter_us > 0 or gain > 0.0
    if manual_exposure and not (
        100 <= shutter_us <= 50000 and 1.0 <= gain <= 64.0
    ):
        raise ValueError(
            'manual exposure requires shutter [100, 50000] us and gain [1, 64]'
        )
    if manual_exposure and shutter_us > 1_000_000 / framerate:
        raise ValueError('camera shutter cannot exceed one video frame period')
    host = f'[{address}]' if address.version == 6 else str(address)
    output = f'udp://{host}:{port}?pkt_size=1316'
    command = [
        executable,
        '-t',
        '0',
        '-n',
        '--codec',
        'libav',
        '--libav-format',
        'mpegts',
        '--low-latency',
        # Keep the stream aligned with the measured camera/body contract.
        '--rotation',
        '180',
        '--width',
        str(width),
        '--height',
        str(height),
        '--framerate',
        str(framerate),
        '--bitrate',
        str(bitrate_bps),
    ]
    if manual_exposure:
        command.extend(['--shutter', str(shutter_us), '--gain', str(gain)])
    command.extend(['-o', output])
    return command
