#!/usr/bin/env python3
"""Run the Pi camera and panel-only OpenCV worker without CUDA or ONNX."""

from __future__ import annotations

import argparse
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml


def load_camera_profile(path: Path, name: str | None) -> dict:
    """Return one day/night profile, with 'common' merged into its camera."""
    with path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError('camera profile file must be a YAML mapping')
    profiles = document.get('profiles')
    if not isinstance(profiles, dict):
        raise RuntimeError('camera profile file has no "profiles" mapping')
    selected = name or document.get('active')
    if selected not in profiles:
        available = ', '.join(sorted(profiles)) or '(none)'
        raise RuntimeError(
            f'camera profile {selected!r} is not defined; available: {available}'
        )
    profile = profiles[selected] or {}
    camera = dict(document.get('common') or {})
    camera.update(profile.get('camera') or {})
    return {
        'name': selected,
        'camera': camera,
        'detector': dict(profile.get('detector') or {}),
        'survey': dict(profile.get('survey') or {}),
    }


def apply_detector_profile(config: dict, detector: dict) -> list[str]:
    """Overlay the profile's detector keys on the loaded worker config.

    The keys are applied in memory only. laptop_ai.yaml stays the documented
    baseline on disk, and the profile carries the deltas from it, so a profile
    switch can never leave the baseline file half-rewritten.
    """
    section = config.setdefault('panel_detector', {})
    changes = []
    for key, value in detector.items():
        # An unknown key means the profile and laptop_ai.yaml have drifted.
        # Refuse rather than silently applying a setting nothing reads.
        if key not in section:
            raise RuntimeError(
                f'profile detector key {key!r} is not in panel_detector; '
                'the profile and laptop_ai.yaml have drifted apart'
            )
        if section[key] != value:
            changes.append(f'{key}: {section[key]} -> {value}')
        section[key] = value
    return changes


def camera_command(camera: str, port: int, profile: dict) -> list[str]:
    """Return the low-latency local camera stream command for one profile."""
    result = [
        camera,
        '-t',
        '0',
        '-n',
        '--codec',
        'libav',
        '--libav-format',
        'mpegts',
        '--low-latency',
        # The measured camera/body calibration assumes perception receives
        # the physically inverted camera image after this 180-degree rotation.
        '--rotation',
        str(profile['rotation']),
        '--width',
        str(profile['width']),
        '--height',
        str(profile['height']),
        '--framerate',
        str(profile['framerate']),
        '--bitrate',
        str(profile['bitrate']),
    ]
    # A null shutter or gain means the flag is left off, which is what hands
    # exposure back to the camera's own AEC/AGC. Passing 0 would not do that -
    # it would pin the value - so the keys have to be omitted, not defaulted.
    if profile.get('shutter_us') is not None:
        result += ['--shutter', str(profile['shutter_us'])]
    if profile.get('gain') is not None:
        result += ['--gain', str(profile['gain'])]
    for flag in ('awb', 'ev', 'exposure', 'metering', 'denoise'):
        if profile.get(flag) is not None:
            result += [f'--{flag}', str(profile[flag])]
    result += ['-o', f'udp://127.0.0.1:{port}?pkt_size=1316']
    return result


def load_panel_config(project_root: Path, config_path: Path) -> dict:
    """Load the shared protocol config for a loopback panel-only worker."""
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RuntimeError('panel CPU config must be a YAML mapping')
    config['network']['pi_ip'] = '127.0.0.1'
    config['dirt_model']['path'] = ''
    runtime = config.setdefault('runtime', {})
    runtime['optimizer_config'] = ''
    runtime['survey_debug_directory'] = str(
        project_root / 'captures' / 'panel_cpu_survey'
    )
    runtime['survey_debug_interval_s'] = 1.0
    runtime['survey_debug_max_frames'] = 120
    sys.path.insert(0, str(project_root / 'laptop_ai'))
    return config


def parser() -> argparse.ArgumentParser:
    """Build the fixed Raspberry Pi panel-worker command line."""
    project_root = Path(__file__).resolve().parents[1]
    result = argparse.ArgumentParser(
        description='Run CPU-only panel localization perception on the Pi.'
    )
    result.add_argument('--project-root', default=str(project_root))
    result.add_argument(
        '--config',
        default=str(project_root / 'laptop_ai/config/laptop_ai.yaml'),
    )
    result.add_argument(
        '--camera-profiles',
        default=str(project_root / 'config/camera_profiles.yaml'),
    )
    result.add_argument(
        '--profile',
        default=None,
        help="Capture profile to use, or the file's 'active' key",
    )
    result.add_argument('--camera-executable', default='rpicam-vid')
    return result


def main() -> int:
    """Supervise the local camera and CPU-only perception worker."""
    arguments = parser().parse_args()
    project_root = Path(arguments.project_root).expanduser().resolve()
    config_path = Path(arguments.config).expanduser().resolve()
    camera = shutil.which(arguments.camera_executable)
    if camera is None:
        print('ERROR: rpicam-vid is unavailable', file=sys.stderr)
        return 1
    config = load_panel_config(project_root, config_path)
    from laptop_ai.worker import LaptopAiWorker

    camera_process: subprocess.Popen | None = None
    worker = None

    def stop(_signum=None, _frame=None) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        video_port = int(config['video']['port'])
        profile = load_camera_profile(
            Path(arguments.camera_profiles).expanduser().resolve(),
            arguments.profile,
        )
        print(f'Capture profile: {profile["name"]}', flush=True)
        for change in apply_detector_profile(config, profile['detector']):
            print(f'  detector {change}', flush=True)
        camera_process = subprocess.Popen(
            camera_command(camera, video_port, profile['camera'])
        )
        time.sleep(0.8)
        if camera_process.poll() is not None:
            raise RuntimeError(
                f'rpicam-vid exited with {camera_process.returncode}'
            )
        worker = LaptopAiWorker(config, panel_only=True)
        print(
            'Pi CPU panel vision ready; OpenCV only; CUDA/ONNX disabled',
            flush=True,
        )
        worker.run()
    except KeyboardInterrupt:
        return 0
    except (KeyError, OSError, RuntimeError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        if worker is not None:
            worker.close()
        if camera_process is not None and camera_process.poll() is None:
            camera_process.terminate()
            try:
                camera_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                camera_process.kill()
                camera_process.wait(timeout=3.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
