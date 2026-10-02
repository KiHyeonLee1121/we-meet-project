"""OpenCV visualization for the existing laptop perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time
from typing import Mapping, Sequence

import cv2
import numpy as np

from laptop_ai.panel_detector import PanelRectangle


@dataclass(frozen=True)
class VisualizationState:
    """One already-computed worker result to draw without re-running AI."""

    mode: str
    active_panel_id: int
    frame_id: int
    control_connected: bool
    valid: bool
    panel_visible: bool
    dirt_found: bool
    inference_ms: float
    invalid_reason: str
    model_name: str
    pi_ip: str
    target_x_norm: float
    target_y_norm: float
    dirt_values: Mapping[str, float]


def _point(width: int, height: int, x_norm: float, y_norm: float) -> tuple[int, int]:
    x = int(round(max(0.0, min(1.0, x_norm)) * max(0, width - 1)))
    y = int(round(max(0.0, min(1.0, y_norm)) * max(0, height - 1)))
    return x, y


def _normalized_box(
    width: int,
    height: int,
    values: Mapping[str, float],
) -> tuple[tuple[int, int], tuple[int, int]]:
    left, top = _point(
        width,
        height,
        float(values.get('dirt_bbox_x_norm', 0.0)),
        float(values.get('dirt_bbox_y_norm', 0.0)),
    )
    right, bottom = _point(
        width,
        height,
        float(values.get('dirt_bbox_x_norm', 0.0))
        + float(values.get('dirt_bbox_w_norm', 0.0)),
        float(values.get('dirt_bbox_y_norm', 0.0))
        + float(values.get('dirt_bbox_h_norm', 0.0)),
    )
    return (left, top), (right, bottom)


def render_overlay(
    frame: np.ndarray,
    *,
    panels: Sequence[PanelRectangle],
    selected: PanelRectangle | None,
    state: VisualizationState,
    display_fps: float = 0.0,
) -> np.ndarray:
    """Draw worker output on a copy of one decoded Pi camera frame."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('a BGR camera frame is required')
    output = frame.copy()
    height, width = output.shape[:2]

    shade = output.copy()
    header_height = min(height, 82)
    cv2.rectangle(shade, (0, 0), (width, header_height), (12, 16, 20), -1)
    output = cv2.addWeighted(shade, 0.76, output, 0.24, 0.0)

    connected_color = (70, 210, 110) if state.control_connected else (60, 80, 230)
    connection = 'PI LINK OK' if state.control_connected else 'WAITING FOR PI CONTROL'
    cv2.putText(
        output,
        connection,
        (16, 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.66,
        connected_color,
        2,
        cv2.LINE_AA,
    )
    summary = (
        f'MODE {state.mode.upper()}  PANEL {state.active_panel_id}  '
        f'FRAME {state.frame_id}  AI {state.inference_ms:.1f} ms  VIEW {display_fps:.1f} fps'
    )
    cv2.putText(
        output,
        summary,
        (16, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (230, 236, 240),
        1,
        cv2.LINE_AA,
    )
    model_line = f'GPU MODEL {state.model_name}  PI {state.pi_ip}'
    cv2.putText(
        output,
        model_line,
        (16, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (170, 184, 196),
        1,
        cv2.LINE_AA,
    )

    selected_id = selected.candidate_id if selected is not None else None
    for panel in panels:
        is_selected = panel.candidate_id == selected_id
        color = (70, 220, 110) if is_selected else (230, 170, 40)
        thickness = 3 if is_selected else 2
        first = (panel.x, panel.y)
        second = (panel.x + panel.width, panel.y + panel.height)
        cv2.rectangle(output, first, second, color, thickness)
        label = (
            f'TARGET {panel.candidate_id}' if is_selected
            else f'PANEL {panel.candidate_id}'
        )
        cv2.putText(
            output,
            f'{label} {panel.confidence:.2f}',
            (panel.x, max(header_height + 18, panel.y - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            color,
            2,
            cv2.LINE_AA,
        )

    target = _point(width, height, state.target_x_norm, state.target_y_norm)
    cv2.drawMarker(
        output,
        target,
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=22,
        thickness=2,
    )

    if state.dirt_found:
        first, second = _normalized_box(width, height, state.dirt_values)
        cv2.rectangle(output, first, second, (40, 40, 245), 3)
        centroid = _point(
            width,
            height,
            float(state.dirt_values.get('dirt_centroid_x_norm', 0.0)),
            float(state.dirt_values.get('dirt_centroid_y_norm', 0.0)),
        )
        cv2.drawMarker(
            output,
            centroid,
            (40, 40, 245),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=24,
            thickness=3,
        )
        confidence = float(state.dirt_values.get('dirt_confidence', 0.0))
        cv2.putText(
            output,
            f'DIRT {confidence:.2f}',
            (first[0], max(header_height + 20, first[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (40, 40, 245),
            2,
            cv2.LINE_AA,
        )

    if state.mode == 'idle':
        status = 'MISSION IDLE - camera preview only'
        status_color = (90, 210, 240)
    elif not state.valid:
        status = f'AI RESULT BLOCKED - {state.invalid_reason or "invalid result"}'
        status_color = (60, 90, 240)
    elif state.dirt_found:
        status = 'DIRT DETECTED'
        status_color = (60, 80, 245)
    elif state.panel_visible:
        status = 'PANEL DETECTED - NO DIRT'
        status_color = (70, 210, 110)
    else:
        status = 'NO PANEL DETECTED'
        status_color = (90, 210, 240)
    cv2.rectangle(output, (0, max(0, height - 37)), (width, height), (12, 16, 20), -1)
    cv2.putText(
        output,
        status,
        (16, height - 13),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        status_color,
        2,
        cv2.LINE_AA,
    )
    return output


class OpenCvViewer:
    """Resizable desktop window driven by the worker's existing frame loop."""

    def __init__(
        self,
        *,
        window_title: str = 'DA-DAKA Laptop AI Monitor',
        screenshot_directory: str | Path = 'logs/laptop_ai_viewer',
        fullscreen: bool = False,
    ) -> None:
        self.window_title = window_title
        self.screenshot_directory = Path(screenshot_directory)
        self.fullscreen = fullscreen
        self._window_created = False
        self._last_frame_s: float | None = None
        self._display_fps = 0.0
        self._last_rendered: np.ndarray | None = None

    def show(
        self,
        frame: np.ndarray,
        *,
        panels: Sequence[PanelRectangle],
        selected: PanelRectangle | None,
        state: VisualizationState,
    ) -> bool:
        """Render one frame; return false when the user requests exit."""
        now_s = time.monotonic()
        if self._last_frame_s is not None:
            instant = 1.0 / max(now_s - self._last_frame_s, 1e-6)
            self._display_fps = instant if self._display_fps <= 0.0 else (
                0.90 * self._display_fps + 0.10 * instant
            )
        self._last_frame_s = now_s
        rendered = render_overlay(
            frame,
            panels=panels,
            selected=selected,
            state=state,
            display_fps=self._display_fps,
        )
        self._last_rendered = rendered
        if not self._window_created:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_title, 1280, 760)
            if self.fullscreen:
                cv2.setWindowProperty(
                    self.window_title,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            self._window_created = True
        cv2.imshow(self.window_title, rendered)
        key = cv2.waitKey(1) & 0xFF
        if key in {27, ord('q')}:
            return False
        if key == ord('s'):
            self.save_screenshot()
        if key == ord('f'):
            self.fullscreen = not self.fullscreen
            value = cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL
            cv2.setWindowProperty(self.window_title, cv2.WND_PROP_FULLSCREEN, value)
        try:
            return cv2.getWindowProperty(self.window_title, cv2.WND_PROP_VISIBLE) >= 1
        except cv2.error:
            return True

    def save_screenshot(self) -> Path | None:
        """Save the current annotated frame when one is available."""
        if self._last_rendered is None:
            return None
        self.screenshot_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        path = self.screenshot_directory / f'da-daka-{timestamp}.jpg'
        if not cv2.imwrite(str(path), self._last_rendered):
            raise RuntimeError(f'failed to write screenshot: {path}')
        return path

    def close(self) -> None:
        """Close only this monitor window."""
        if not self._window_created:
            return
        try:
            cv2.destroyWindow(self.window_title)
            cv2.waitKey(1)
        except cv2.error:
            pass
        self._window_created = False

