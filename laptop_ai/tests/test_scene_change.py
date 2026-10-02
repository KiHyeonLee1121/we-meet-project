from __future__ import annotations

import numpy as np

from laptop_ai.scene_change import SceneChangeDetector


def test_static_scene_does_not_trigger() -> None:
    detector = SceneChangeDetector(threshold=0.1, sample_step=2)
    frame = np.full((32, 32, 3), 120, dtype=np.uint8)
    first = detector.update(frame)
    second = detector.update(frame.copy())
    assert first.significant is False
    assert second.combined_score == 0.0
    assert second.significant is False


def test_large_lighting_change_triggers() -> None:
    detector = SceneChangeDetector(threshold=0.1, sample_step=2)
    detector.update(np.zeros((32, 32, 3), dtype=np.uint8))
    sample = detector.update(np.full((32, 32, 3), 255, dtype=np.uint8))
    assert sample.motion_score > 0.9
    assert sample.histogram_score > 0.9
    assert sample.significant is True


def test_centroid_drift_contributes() -> None:
    detector = SceneChangeDetector(threshold=0.05, sample_step=2)
    frame = np.full((32, 32, 3), 100, dtype=np.uint8)
    detector.update(frame, (0.1, 0.1))
    sample = detector.update(frame, (0.9, 0.9))
    assert sample.centroid_drift > 0.7
    assert sample.significant is True
