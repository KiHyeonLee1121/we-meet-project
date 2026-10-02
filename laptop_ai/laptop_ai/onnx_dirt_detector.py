"""CUDA ONNX segmentation adapter for dirt inside one panel ROI."""

from dataclasses import dataclass

import cv2
import numpy as np
import onnxruntime as ort

from laptop_ai.runtime_tuning import (
    RuntimeTuning,
    configure_cuda_environment,
    create_session_options,
    cuda_provider,
    resolve_model_path,
)


@dataclass(frozen=True)
class DirtMaskResult:
    centroid_x_norm: float
    centroid_y_norm: float
    bbox_x_norm: float
    bbox_y_norm: float
    bbox_w_norm: float
    bbox_h_norm: float
    confidence: float


class OnnxDirtSegmenter:
    """Run a binary segmentation model through NVIDIA CUDAExecutionProvider."""

    def __init__(
        self,
        model_path: str,
        *,
        input_width: int,
        input_height: int,
        threshold: float,
        minimum_area_ratio: float,
        output_channel: int = 0,
        performance: dict | None = None,
    ) -> None:
        path = resolve_model_path(model_path)
        if min(input_width, input_height) <= 0:
            raise ValueError('model input dimensions must be positive')
        if not 0.0 < threshold < 1.0:
            raise ValueError('segmentation threshold must be within (0, 1)')
        if not 0.0 < minimum_area_ratio < 1.0:
            raise ValueError('minimum_area_ratio must be within (0, 1)')
        providers = ort.get_available_providers()
        if 'CUDAExecutionProvider' not in providers:
            raise RuntimeError(
                'onnxruntime-gpu CUDAExecutionProvider is unavailable; '
                f'providers={providers}'
            )
        self.tuning = RuntimeTuning.from_mapping(performance)
        configure_cuda_environment(self.tuning)
        self.session = ort.InferenceSession(
            str(path),
            sess_options=create_session_options(ort, self.tuning),
            providers=[cuda_provider(self.tuning), 'CPUExecutionProvider'],
        )
        if self.session.get_providers()[0] != 'CUDAExecutionProvider':
            raise RuntimeError('ONNX Runtime did not activate CUDAExecutionProvider')
        self.input_name = self.session.get_inputs()[0].name
        self.input_width = input_width
        self.input_height = input_height
        self.threshold = threshold
        self.minimum_area_ratio = minimum_area_ratio
        self.output_channel = output_channel
        self.model_name = path.name
        self._warmup_remaining = self.tuning.onnx_warmup_runs

    def detect(self, bgr_roi: np.ndarray) -> DirtMaskResult | None:
        if bgr_roi is None or bgr_roi.size == 0:
            raise ValueError('non-empty panel ROI is required')
        resized = cv2.resize(
            bgr_roi,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        while self._warmup_remaining > 0:
            self.session.run(None, {self.input_name: tensor})
            self._warmup_remaining -= 1
        output = np.asarray(self.session.run(None, {self.input_name: tensor})[0])
        probability = self._probability_map(output)
        mask = probability >= self.threshold
        area_ratio = float(mask.mean())
        if area_ratio < self.minimum_area_ratio:
            return None
        ys, xs = np.nonzero(mask)
        x_min = int(xs.min())
        x_max = int(xs.max())
        y_min = int(ys.min())
        y_max = int(ys.max())
        height, width = probability.shape
        return DirtMaskResult(
            centroid_x_norm=float(xs.mean() / width),
            centroid_y_norm=float(ys.mean() / height),
            bbox_x_norm=x_min / width,
            bbox_y_norm=y_min / height,
            bbox_w_norm=(x_max - x_min + 1) / width,
            bbox_h_norm=(y_max - y_min + 1) / height,
            confidence=float(probability[mask].mean()),
        )

    def _probability_map(self, output: np.ndarray) -> np.ndarray:
        while output.ndim > 2 and output.shape[0] == 1:
            output = output[0]
        if output.ndim == 3:
            if not 0 <= self.output_channel < output.shape[0]:
                raise RuntimeError('configured output_channel is outside model output')
            output = output[self.output_channel]
        if output.ndim != 2:
            raise RuntimeError(
                f'expected a binary segmentation map, got shape {output.shape}'
            )
        output = output.astype(np.float32)
        if float(output.min()) < 0.0 or float(output.max()) > 1.0:
            output = 1.0 / (1.0 + np.exp(-np.clip(output, -30.0, 30.0)))
        return output
