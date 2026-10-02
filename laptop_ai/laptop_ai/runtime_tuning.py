"""Validated OpenCV and ONNX Runtime tuning for the laptop GPU worker."""

from dataclasses import dataclass, fields
import os
from pathlib import Path


@dataclass(frozen=True)
class RuntimeTuning:
    opencv_num_threads: int = 2
    opencv_use_opencl: bool = False
    opencv_use_optimized: bool = True
    onnx_intra_op_threads: int = 1
    onnx_inter_op_threads: int = 1
    onnx_graph_optimization: str = 'all'
    onnx_device_id: int = 0
    onnx_warmup_runs: int = 5
    onnx_cuda_conv_use_max_workspace: bool = True
    onnx_cuda_use_tf32: bool = True
    onnx_cuda_arena_extend_strategy: str = 'kNextPowerOfTwo'
    onnx_cuda_cudnn_conv_algo_search: str = 'EXHAUSTIVE'
    cuda_module_loading_lazy: bool = True

    @classmethod
    def from_mapping(cls, values: dict | None) -> 'RuntimeTuning':
        raw = {} if values is None else dict(values)
        allowed = {item.name for item in fields(cls)}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f'unknown performance settings: {unknown}')
        result = cls(**raw)
        result.validate()
        return result

    def validate(self) -> None:
        counts = (
            self.opencv_num_threads,
            self.onnx_intra_op_threads,
            self.onnx_inter_op_threads,
            self.onnx_device_id,
            self.onnx_warmup_runs,
        )
        if any(isinstance(value, bool) or value < 0 for value in counts):
            raise ValueError('runtime thread/device/warmup values cannot be negative')
        if self.onnx_graph_optimization not in {
            'disabled', 'basic', 'extended', 'all'
        }:
            raise ValueError('invalid ONNX graph optimization level')
        if self.onnx_cuda_arena_extend_strategy not in {
            'kNextPowerOfTwo', 'kSameAsRequested'
        }:
            raise ValueError('invalid CUDA arena extension strategy')
        if self.onnx_cuda_cudnn_conv_algo_search not in {
            'EXHAUSTIVE', 'HEURISTIC', 'DEFAULT'
        }:
            raise ValueError('invalid cuDNN convolution search mode')


def configure_cuda_environment(config: RuntimeTuning) -> None:
    """Set CUDA process knobs before ONNX Runtime creates its first context."""
    if config.cuda_module_loading_lazy:
        os.environ.setdefault('CUDA_MODULE_LOADING', 'LAZY')


def configure_opencv(config: RuntimeTuning) -> None:
    """Apply process-wide OpenCV CPU settings once at worker startup."""
    import cv2

    if config.opencv_num_threads > 0:
        cv2.setNumThreads(config.opencv_num_threads)
    cv2.setUseOptimized(config.opencv_use_optimized)
    cv2.ocl.setUseOpenCL(config.opencv_use_opencl)


def create_session_options(ort, config: RuntimeTuning):
    options = ort.SessionOptions()
    if config.onnx_intra_op_threads > 0:
        options.intra_op_num_threads = config.onnx_intra_op_threads
    if config.onnx_inter_op_threads > 0:
        options.inter_op_num_threads = config.onnx_inter_op_threads
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    levels = {
        'disabled': ort.GraphOptimizationLevel.ORT_DISABLE_ALL,
        'basic': ort.GraphOptimizationLevel.ORT_ENABLE_BASIC,
        'extended': ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED,
        'all': ort.GraphOptimizationLevel.ORT_ENABLE_ALL,
    }
    options.graph_optimization_level = levels[config.onnx_graph_optimization]
    return options


def cuda_provider(config: RuntimeTuning):
    """Return fail-closed CUDA provider options for fixed-shape inference."""
    return (
        'CUDAExecutionProvider',
        {
            'device_id': str(config.onnx_device_id),
            'do_copy_in_default_stream': '1',
            'arena_extend_strategy': config.onnx_cuda_arena_extend_strategy,
            'cudnn_conv_algo_search': config.onnx_cuda_cudnn_conv_algo_search,
            'cudnn_conv_use_max_workspace': (
                '1' if config.onnx_cuda_conv_use_max_workspace else '0'
            ),
            'use_tf32': '1' if config.onnx_cuda_use_tf32 else '0',
        },
    )


def resolve_model_path(path: str) -> Path:
    result = Path(path).expanduser().resolve()
    if not result.is_file():
        raise FileNotFoundError(f'ONNX model not found: {result}')
    return result
