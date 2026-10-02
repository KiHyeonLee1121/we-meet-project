"""Lightweight scene-change signal for optimization re-evaluation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class SceneChangeSample:
    motion_score: float
    histogram_score: float
    centroid_drift: float
    combined_score: float
    significant: bool


class SceneChangeDetector:
    """Approximate Pendulum's motion/histogram/bbox-drift trigger cheaply.

    It does not change flight state and is safe to run as a best-effort AI-side
    signal. Frames are spatially subsampled before analysis to keep overhead low.
    """

    def __init__(self, threshold: float = 0.20, sample_step: int = 8) -> None:
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("threshold must be within [0, 1]")
        if sample_step < 1:
            raise ValueError("sample_step must be positive")
        self.threshold = threshold
        self.sample_step = sample_step
        self._previous_gray: np.ndarray | None = None
        self._previous_hist: np.ndarray | None = None
        self._previous_centroid: tuple[float, float] | None = None

    @staticmethod
    def _histogram(gray: np.ndarray) -> np.ndarray:
        hist, _ = np.histogram(gray, bins=16, range=(0.0, 255.0))
        hist = hist.astype(np.float32)
        total = float(hist.sum())
        if total > 0.0:
            hist /= total
        return hist

    def update(
        self,
        frame: np.ndarray,
        centroid_norm: tuple[float, float] | None = None,
    ) -> SceneChangeSample:
        if frame.ndim == 3:
            sampled = frame[:: self.sample_step, :: self.sample_step, :3].astype(np.float32)
            gray = sampled.mean(axis=2)
        elif frame.ndim == 2:
            gray = frame[:: self.sample_step, :: self.sample_step].astype(np.float32)
        else:
            raise ValueError("frame must be HxW or HxWxC")

        hist = self._histogram(gray)
        motion = 0.0
        histogram = 0.0
        if self._previous_gray is not None and self._previous_gray.shape == gray.shape:
            motion = float(np.mean(np.abs(gray - self._previous_gray)) / 255.0)
        if self._previous_hist is not None:
            histogram = float(0.5 * np.abs(hist - self._previous_hist).sum())

        drift = 0.0
        if centroid_norm is not None and self._previous_centroid is not None:
            dx = centroid_norm[0] - self._previous_centroid[0]
            dy = centroid_norm[1] - self._previous_centroid[1]
            drift = min((dx * dx + dy * dy) ** 0.5 / (2.0**0.5), 1.0)

        combined = min(0.45 * motion + 0.35 * histogram + 0.20 * drift, 1.0)
        sample = SceneChangeSample(
            motion_score=motion,
            histogram_score=histogram,
            centroid_drift=drift,
            combined_score=combined,
            significant=combined >= self.threshold,
        )
        self._previous_gray = gray
        self._previous_hist = hist
        if centroid_norm is not None:
            self._previous_centroid = centroid_norm
        return sample
