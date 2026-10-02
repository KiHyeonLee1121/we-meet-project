import pytest

from laptop_ai.runtime_tuning import RuntimeTuning


def test_runtime_tuning_rejects_unknown_and_invalid_values():
    with pytest.raises(ValueError, match='unknown'):
        RuntimeTuning.from_mapping({'invented': True})
    with pytest.raises(ValueError, match='cannot be negative'):
        RuntimeTuning.from_mapping({'onnx_warmup_runs': -1})


def test_runtime_tuning_loads_cuda_profile():
    config = RuntimeTuning.from_mapping({
        'onnx_device_id': 1,
        'onnx_cuda_cudnn_conv_algo_search': 'HEURISTIC',
    })
    assert config.onnx_device_id == 1
    assert config.onnx_cuda_cudnn_conv_algo_search == 'HEURISTIC'
