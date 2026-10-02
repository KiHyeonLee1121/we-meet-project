import numpy as np

from laptop_ai.panel_detector import PanelRectangle
from laptop_ai.visualization import VisualizationState, render_overlay


def state(**overrides):
    values = {
        'mode': 'clean',
        'active_panel_id': 2,
        'frame_id': 41,
        'control_connected': True,
        'valid': True,
        'panel_visible': True,
        'dirt_found': True,
        'inference_ms': 7.5,
        'invalid_reason': '',
        'model_name': 'dirt.onnx',
        'pi_ip': '192.168.1.20',
        'target_x_norm': 0.5,
        'target_y_norm': 0.5,
        'dirt_values': {
            'dirt_centroid_x_norm': 0.55,
            'dirt_centroid_y_norm': 0.58,
            'dirt_bbox_x_norm': 0.45,
            'dirt_bbox_y_norm': 0.48,
            'dirt_bbox_w_norm': 0.20,
            'dirt_bbox_h_norm': 0.18,
            'dirt_confidence': 0.91,
        },
    }
    values.update(overrides)
    return VisualizationState(**values)


def test_render_overlay_draws_without_modifying_camera_frame():
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    original = frame.copy()
    panel = PanelRectangle(1, 220, 120, 210, 150, 0.88)

    rendered = render_overlay(
        frame,
        panels=[panel],
        selected=panel,
        state=state(),
        display_fps=19.8,
    )

    assert rendered.shape == frame.shape
    assert np.array_equal(frame, original)
    assert np.count_nonzero(rendered) > 0
    assert rendered[120, 220].any()


def test_render_overlay_handles_idle_preview_without_detections():
    frame = np.full((240, 320, 3), 25, dtype=np.uint8)

    rendered = render_overlay(
        frame,
        panels=[],
        selected=None,
        state=state(
            mode='idle',
            active_panel_id=-1,
            control_connected=False,
            valid=False,
            panel_visible=False,
            dirt_found=False,
            dirt_values={},
            invalid_reason='mission-idle-or-control-stale',
        ),
    )

    assert rendered.shape == frame.shape
    assert not np.array_equal(rendered, frame)


def test_render_overlay_rejects_non_bgr_input():
    frame = np.zeros((120, 160), dtype=np.uint8)

    try:
        render_overlay(
            frame,
            panels=[],
            selected=None,
            state=state(dirt_found=False, dirt_values={}),
        )
    except ValueError as exc:
        assert 'BGR' in str(exc)
    else:
        raise AssertionError('grayscale frame should be rejected')

