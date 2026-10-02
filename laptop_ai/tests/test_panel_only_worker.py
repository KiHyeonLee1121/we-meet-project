"""Tests for the Raspberry Pi CPU-only panel perception path."""

import numpy as np

from laptop_ai.panel_detector import PanelRectangle
from laptop_ai.worker import LaptopAiWorker


class FakePanelDetector:
    """Return one stable rectangle without invoking OpenCV."""

    @staticmethod
    def detect(_frame):
        return [PanelRectangle(1, 20, 20, 60, 40, 0.9)]


def test_panel_only_clean_mode_never_invokes_dirt_model():
    """Reacquisition stays valid without constructing an ONNX detector."""
    worker = LaptopAiWorker.__new__(LaptopAiWorker)
    worker.panel_detector = FakePanelDetector()
    worker.target_x_norm = 0.5
    worker.target_y_norm = 0.5
    worker.maximum_target_distance_norm = 0.5
    worker.scene_change = None
    worker.dirt_detector = None
    worker.model_name = 'opencv-panel-cpu'
    sent = []
    worker._send = lambda **values: sent.append(values)
    worker._show_frame = lambda *_args, **_values: True

    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    assert worker._process_frame(frame, 'clean', 7)

    assert sent[0]['valid'] is True
    assert sent[0]['panel_visible'] is True
    assert sent[0]['dirt_found'] is False
    assert sent[0]['panels'][0]['candidate_id'] == 1


def test_panel_only_survey_saves_sparse_debug_frame(tmp_path):
    """A live survey stores an inspectable frame with its candidate count."""
    worker = LaptopAiWorker.__new__(LaptopAiWorker)
    worker.panel_detector = FakePanelDetector()
    worker.target_x_norm = 0.5
    worker.target_y_norm = 0.5
    worker.maximum_target_distance_norm = 0.5
    worker.scene_change = None
    worker.dirt_detector = None
    worker.model_name = 'opencv-panel-cpu'
    worker.session_id = 'test-session'
    worker.frame_id = 17
    worker.survey_debug_directory = tmp_path
    worker.survey_debug_interval_s = 1.0
    worker.survey_debug_max_frames = 2
    worker._survey_debug_last_s = -float('inf')
    worker._survey_debug_saved = 0
    sent = []
    worker._send = lambda **values: sent.append(values)
    worker._show_frame = lambda *_args, **_values: True

    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    assert worker._process_frame(frame, 'survey', -1)

    saved = list((tmp_path / 'test-session').glob('*.jpg'))
    assert len(saved) == 1
    assert saved[0].name == 'frame_000017_panels_01.jpg'
    assert worker._survey_debug_saved == 1
    assert sent[0]['panel_visible'] is True
