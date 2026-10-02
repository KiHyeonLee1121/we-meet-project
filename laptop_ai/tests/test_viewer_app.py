from pathlib import Path

import yaml

from laptop_ai.viewer_app import load_config


def test_load_config_applies_runtime_overrides(tmp_path: Path):
    path = tmp_path / 'laptop.yaml'
    path.write_text(
        yaml.safe_dump({
            'network': {'pi_ip': '192.0.2.1'},
            'dirt_model': {'path': 'old.onnx'},
        }),
        encoding='utf-8',
    )

    config = load_config(
        path,
        pi_ip='198.51.100.20',
        model_path='models/new.onnx',
    )

    assert config['network']['pi_ip'] == '198.51.100.20'
    assert config['dirt_model']['path'] == 'models/new.onnx'

