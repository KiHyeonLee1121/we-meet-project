"""Tests for the classical panel rectangle detector."""

import cv2
import numpy as np
import pytest

from laptop_ai.panel_detector import PanelDetector


def detector(epsilon: float = 0.035) -> PanelDetector:
    """Build the production-shaped detector with one configurable value."""
    return PanelDetector(
        minimum_area_ratio=0.01,
        maximum_area_ratio=0.90,
        minimum_aspect_ratio=1.0,
        maximum_aspect_ratio=4.0,
        maximum_panels=32,
        polygon_epsilon_ratio=epsilon,
    )


def test_grid_panel_is_detected_as_one_outer_rectangle():
    """Internal cell lines must not prevent the outer panel detection."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    corners = np.array([[230, 130], [365, 138], [355, 245], [220, 235]])
    cv2.polylines(frame, [corners], True, (180, 180, 180), 5)
    for ratio in (0.25, 0.50, 0.75):
        top = corners[0] + ratio * (corners[1] - corners[0])
        bottom = corners[3] + ratio * (corners[2] - corners[3])
        cv2.line(frame, top.astype(int), bottom.astype(int), (180, 180, 180), 4)
    for ratio in (0.25, 0.50, 0.75):
        left = corners[0] + ratio * (corners[3] - corners[0])
        right = corners[1] + ratio * (corners[2] - corners[1])
        cv2.line(frame, left.astype(int), right.astype(int), (180, 180, 180), 4)

    panels = detector().detect(frame)

    assert len(panels) == 1
    assert panels[0].confidence >= 0.55


def test_low_contrast_blue_night_panel_uses_color_fallback():
    """Blue chroma must recover a panel with almost no grayscale edge."""
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    corners = np.array([[220, 150], [405, 165], [390, 300], [205, 280]])
    cv2.fillConvexPoly(frame, corners, (48, 34, 28))
    for ratio in (0.20, 0.40, 0.60, 0.80):
        top = corners[0] + ratio * (corners[1] - corners[0])
        bottom = corners[3] + ratio * (corners[2] - corners[3])
        cv2.line(frame, top.astype(int), bottom.astype(int), (58, 44, 38), 2)
    for ratio in (0.33, 0.66):
        left = corners[0] + ratio * (corners[3] - corners[0])
        right = corners[1] + ratio * (corners[2] - corners[1])
        cv2.line(frame, left.astype(int), right.astype(int), (58, 44, 38), 2)

    panels = detector().detect(frame)

    assert len(panels) == 1
    assert panels[0].confidence >= 0.80


def test_blue_fallback_rejects_noise_and_fixed_top_left_structure():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (110, 70), (55, 35, 25), -1)
    frame[350:352, 500:502] = (80, 20, 10)

    assert detector().detect(frame) == []


def test_partial_panel_touching_image_edge_is_rejected():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (560, 100), (639, 260), (55, 35, 25), -1)

    assert detector().detect(frame) == []


def test_full_height_blue_background_strip_is_rejected():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (500, 0), (639, 479), (55, 35, 25), -1)

    assert detector().detect(frame) == []


def test_blue_road_rectangle_without_repeated_grid_is_rejected():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (180, 140), (440, 330), (48, 34, 28), -1)
    cv2.line(frame, (180, 235), (440, 235), (70, 65, 30), 6)

    assert detector().detect(frame) == []


def test_low_texture_blue_ground_with_weak_bands_is_rejected():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (180, 140), (440, 330), (48, 34, 28), -1)
    for x in (220, 260, 300, 340, 380, 420):
        cv2.line(frame, (x, 140), (x, 330), (49, 35, 29), 7)
    for y in (190, 240, 290):
        cv2.line(frame, (180, y), (440, y), (49, 35, 29), 7)

    assert detector().detect(frame) == []


def test_portrait_blue_lane_region_is_rejected_even_with_crossing_lines():
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    cv2.rectangle(frame, (350, 120), (470, 360), (48, 34, 28), -1)
    for x in (375, 400, 425, 450):
        cv2.line(frame, (x, 120), (x, 360), (58, 44, 38), 2)
    for y in (180, 240, 300):
        cv2.line(frame, (350, y), (470, y), (58, 44, 38), 2)

    assert detector().detect(frame) == []


@pytest.mark.parametrize('value', [0.0, -0.01, 0.1])
def test_polygon_epsilon_ratio_is_bounded(value):
    with pytest.raises(ValueError, match='epsilon'):
        detector(value)
