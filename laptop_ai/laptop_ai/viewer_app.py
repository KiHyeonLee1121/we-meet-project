"""User-facing DA-DAKA laptop AI monitor entry point."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import yaml

from laptop_ai.visualization import OpenCvViewer


def default_config_path() -> Path:
    """Return the configuration shipped beside the editable package."""
    return Path(__file__).resolve().parents[1] / 'config' / 'laptop_ai.yaml'


def load_config(
    path: str | Path,
    *,
    pi_ip: str | None = None,
    model_path: str | None = None,
) -> dict:
    """Load the worker configuration with optional safe CLI overrides."""
    config_path = Path(path).expanduser().resolve()
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('laptop AI config must be a YAML mapping')
    if pi_ip:
        config['network']['pi_ip'] = pi_ip
    if model_path:
        config['dirt_model']['path'] = str(Path(model_path).expanduser())
    return config


def parser() -> argparse.ArgumentParser:
    """Build a concise desktop-monitor command line."""
    result = argparse.ArgumentParser(
        description=(
            'Show the Pi camera and the exact panel/dirt results already '
            'produced by the DA-DAKA NVIDIA worker.'
        )
    )
    result.add_argument('--config', default=str(default_config_path()))
    result.add_argument('--pi-ip', help='override network.pi_ip without editing YAML')
    result.add_argument('--model', help='override dirt_model.path without editing YAML')
    result.add_argument(
        '--fullscreen',
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    result.add_argument('--window-title')
    result.add_argument('--screenshot-directory')
    return result


def main() -> int:
    """Run one integrated worker and monitor window."""
    arguments = parser().parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    try:
        config = load_config(
            arguments.config,
            pi_ip=arguments.pi_ip,
            model_path=arguments.model,
        )
        viewer_config = config.get('viewer', {})
        fullscreen = (
            bool(viewer_config.get('fullscreen', False))
            if arguments.fullscreen is None
            else arguments.fullscreen
        )
        viewer = OpenCvViewer(
            window_title=(
                arguments.window_title
                or str(viewer_config.get(
                    'window_title', 'DA-DAKA Laptop AI Monitor'
                ))
            ),
            screenshot_directory=(
                arguments.screenshot_directory
                or str(viewer_config.get(
                    'screenshot_directory', 'logs/laptop_ai_viewer'
                ))
            ),
            fullscreen=fullscreen,
        )
        # Keep config/overlay unit tests independent from PyAV. The video
        # worker and its production dependencies are required only when the
        # application actually starts.
        from laptop_ai.worker import LaptopAiWorker

        worker = LaptopAiWorker(config, viewer=viewer)
    except (KeyError, OSError, RuntimeError, ValueError) as exc:
        logging.getLogger('laptop_ai.viewer').error('startup failed: %s', exc)
        return 1

    logging.getLogger('laptop_ai.viewer').info(
        'monitor started; Q/ESC=quit, S=screenshot, F=fullscreen'
    )
    try:
        worker.run()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as exc:
        logging.getLogger('laptop_ai.viewer').error('monitor stopped: %s', exc)
        return 1
    finally:
        worker.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
