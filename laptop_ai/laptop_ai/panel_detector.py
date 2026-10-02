"""Classical multi-panel rectangle detector for the 3 m survey image."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class PanelRectangle:
    candidate_id: int
    x: int
    y: int
    width: int
    height: int
    confidence: float

    def normalized(self, image_width: int, image_height: int) -> dict:
        return {
            'candidate_id': self.candidate_id,
            'center_x_norm': (self.x + self.width / 2.0) / image_width,
            'center_y_norm': (self.y + self.height / 2.0) / image_height,
            'width_norm': self.width / image_width,
            'height_norm': self.height / image_height,
            'confidence': self.confidence,
        }


def select_panel_nearest_target(
    panels: list[PanelRectangle],
    *,
    image_width: int,
    image_height: int,
    target_x_norm: float,
    target_y_norm: float,
    maximum_distance_norm: float,
) -> PanelRectangle | None:
    """Select the panel nearest the expected camera/nozzle target point."""
    values = (target_x_norm, target_y_norm, maximum_distance_norm)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('panel target selection values must be finite')
    if min(image_width, image_height) <= 0:
        raise ValueError('image dimensions must be positive')
    if not 0.0 <= target_x_norm <= 1.0 or not 0.0 <= target_y_norm <= 1.0:
        raise ValueError('panel target point must be normalized')
    if not 0.0 < maximum_distance_norm <= math.sqrt(2.0):
        raise ValueError('maximum panel target distance is invalid')
    if not panels:
        return None

    def score(panel: PanelRectangle) -> tuple[float, float, int]:
        center_x = (panel.x + panel.width / 2.0) / image_width
        center_y = (panel.y + panel.height / 2.0) / image_height
        distance = math.hypot(
            center_x - target_x_norm,
            center_y - target_y_norm,
        )
        return distance, -panel.confidence, panel.candidate_id

    selected = min(panels, key=score)
    return selected if score(selected)[0] <= maximum_distance_norm else None


class PanelDetector:
    """Find high-area quadrilateral candidates without assuming their layout."""

    def __init__(
        self,
        *,
        minimum_area_ratio: float,
        maximum_area_ratio: float,
        minimum_aspect_ratio: float,
        maximum_aspect_ratio: float,
        maximum_panels: int,
        polygon_epsilon_ratio: float = 0.035,
        blue_fallback_enabled: bool = True,
        blue_minimum_b_minus_r: int = 12,
        blue_minimum_b_minus_g: int = 0,
        blue_minimum_brightness: int = 20,
        blue_minimum_rectangularity: float = 0.50,
        blue_minimum_landscape_ratio: float = 1.20,
        blue_grid_validation_enabled: bool = True,
        blue_grid_minimum_vertical_lines: int = 3,
        blue_grid_minimum_horizontal_lines: int = 1,
        blue_grid_minimum_edge_density: float = 0.008,
        border_margin_px: int = 2,
        maximum_width_ratio: float = 0.85,
        maximum_height_ratio: float = 0.85,
    ) -> None:
        if not 0.0 < minimum_area_ratio < maximum_area_ratio <= 1.0:
            raise ValueError('panel area ratio bounds are invalid')
        if not 0.0 < minimum_aspect_ratio < maximum_aspect_ratio:
            raise ValueError('panel aspect ratio bounds are invalid')
        if maximum_panels <= 0:
            raise ValueError('maximum_panels must be positive')
        if not 0.0 < polygon_epsilon_ratio < 0.1:
            raise ValueError('polygon epsilon ratio must be within (0, 0.1)')
        if min(
            blue_minimum_b_minus_r,
            blue_minimum_b_minus_g,
            blue_minimum_brightness,
        ) < 0:
            raise ValueError('blue fallback thresholds cannot be negative')
        if not 0.0 < blue_minimum_rectangularity <= 1.0:
            raise ValueError('blue fallback rectangularity is invalid')
        if blue_minimum_landscape_ratio < 1.0:
            raise ValueError('blue fallback landscape ratio is invalid')
        if min(
            blue_grid_minimum_vertical_lines,
            blue_grid_minimum_horizontal_lines,
        ) < 0:
            raise ValueError('blue fallback grid line counts cannot be negative')
        if not 0.0 <= blue_grid_minimum_edge_density <= 1.0:
            raise ValueError('blue fallback grid edge density is invalid')
        if border_margin_px < 0:
            raise ValueError('panel border margin cannot be negative')
        if not 0.0 < maximum_width_ratio <= 1.0:
            raise ValueError('maximum panel width ratio is invalid')
        if not 0.0 < maximum_height_ratio <= 1.0:
            raise ValueError('maximum panel height ratio is invalid')
        self.minimum_area_ratio = minimum_area_ratio
        self.maximum_area_ratio = maximum_area_ratio
        self.minimum_aspect_ratio = minimum_aspect_ratio
        self.maximum_aspect_ratio = maximum_aspect_ratio
        self.maximum_panels = maximum_panels
        self.polygon_epsilon_ratio = polygon_epsilon_ratio
        self.blue_fallback_enabled = bool(blue_fallback_enabled)
        self.blue_minimum_b_minus_r = int(blue_minimum_b_minus_r)
        self.blue_minimum_b_minus_g = int(blue_minimum_b_minus_g)
        self.blue_minimum_brightness = int(blue_minimum_brightness)
        self.blue_minimum_rectangularity = float(blue_minimum_rectangularity)
        self.blue_minimum_landscape_ratio = float(
            blue_minimum_landscape_ratio
        )
        self.blue_grid_validation_enabled = bool(
            blue_grid_validation_enabled
        )
        self.blue_grid_minimum_vertical_lines = int(
            blue_grid_minimum_vertical_lines
        )
        self.blue_grid_minimum_horizontal_lines = int(
            blue_grid_minimum_horizontal_lines
        )
        self.blue_grid_minimum_edge_density = float(
            blue_grid_minimum_edge_density
        )
        self.border_margin_px = int(border_margin_px)
        self.maximum_width_ratio = float(maximum_width_ratio)
        self.maximum_height_ratio = float(maximum_height_ratio)

    @staticmethod
    def _overlap_ratio(first, second) -> float:
        """Return intersection over the smaller box for duplicate suppression."""
        _area, x1, y1, w1, h1, _confidence = first
        _area, x2, y2, w2, h2, _confidence = second
        left = max(x1, x2)
        top = max(y1, y2)
        right = min(x1 + w1, x2 + w2)
        bottom = min(y1 + h1, y2 + h2)
        intersection = max(0, right - left) * max(0, bottom - top)
        return intersection / float(min(w1 * h1, w2 * h2))

    @staticmethod
    def _profile_peak_count(profile: np.ndarray) -> int:
        """Count separated strong bands in a one-dimensional edge profile."""
        if profile.size < 5:
            return 0
        smoothed = np.convolve(
            profile.astype(np.float32),
            np.ones(5, dtype=np.float32) / 5.0,
            mode='same',
        )
        threshold = float(np.median(smoothed) + 1.25 * np.std(smoothed))
        above = smoothed > threshold
        starts = above & ~np.concatenate(([False], above[:-1]))
        return int(np.count_nonzero(starts))

    def _has_repeated_panel_grid(
        self,
        frame: np.ndarray,
        x: int,
        y: int,
        width: int,
        height: int,
    ) -> bool:
        """Identify repeated solar-cell boundaries inside a blue candidate."""
        roi = cv2.cvtColor(
            frame[y:y + height, x:x + width],
            cv2.COLOR_BGR2GRAY,
        )
        padding = max(2, int(min(roi.shape) * 0.07))
        if min(roi.shape) <= 2 * padding:
            return False
        core = roi[padding:-padding, padding:-padding]
        equalized = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(4, 4),
        ).apply(core)
        gradient_x = np.abs(
            cv2.Sobel(equalized, cv2.CV_32F, 1, 0, ksize=3)
        )
        gradient_y = np.abs(
            cv2.Sobel(equalized, cv2.CV_32F, 0, 1, ksize=3)
        )
        edge_density = float(
            np.count_nonzero(cv2.Canny(equalized, 40, 100))
            / equalized.size
        )
        vertical_lines = self._profile_peak_count(gradient_x.mean(axis=0))
        horizontal_lines = self._profile_peak_count(gradient_y.mean(axis=1))
        return bool(
            edge_density >= self.blue_grid_minimum_edge_density
            and
            vertical_lines >= self.blue_grid_minimum_vertical_lines
            and horizontal_lines >= self.blue_grid_minimum_horizontal_lines
        )

    def _blue_candidates(self, frame: np.ndarray, frame_area: float):
        """Find low-contrast blue panels that night-time edge detection misses."""
        blue, green, red = cv2.split(frame.astype(np.int16))
        mask = (
            (blue >= self.blue_minimum_brightness)
            & (blue - red >= self.blue_minimum_b_minus_r)
            & (blue - green >= self.blue_minimum_b_minus_g)
        ).astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((11, 11), dtype=np.uint8),
        )
        contours, _hierarchy = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_ratio = area / frame_area
            if not self.minimum_area_ratio <= area_ratio <= self.maximum_area_ratio:
                continue
            (_center, (rotated_width, rotated_height), _angle) = cv2.minAreaRect(
                contour
            )
            if min(rotated_width, rotated_height) <= 0.0:
                continue
            aspect = max(rotated_width, rotated_height) / min(
                rotated_width, rotated_height
            )
            if not self.minimum_aspect_ratio <= aspect <= self.maximum_aspect_ratio:
                continue
            rectangle_area = rotated_width * rotated_height
            rectangularity = min(1.0, area / rectangle_area)
            if rectangularity < self.blue_minimum_rectangularity:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            # This fixed downward survey profile receives a 180-degree camera
            # rotation, so the installed landscape panels remain wider than
            # they are tall. Road markings produced every portrait candidate
            # in the 2026-08-22 night captures.
            if box_width / float(box_height) < self.blue_minimum_landscape_ratio:
                continue
            if (
                self.blue_grid_validation_enabled
                and not self._has_repeated_panel_grid(
                    frame,
                    x,
                    y,
                    box_width,
                    box_height,
                )
            ):
                continue
            # The Pi camera housing/airframe occupies this fixed image corner
            # in the downward night profile and otherwise looks blue.
            if y <= 1 and x < frame.shape[1] * 0.35:
                continue
            confidence = max(0.01, min(1.0, rectangularity))
            candidates.append(
                (area, x, y, box_width, box_height, confidence)
            )
        return candidates

    def detect(self, frame: np.ndarray) -> list[PanelRectangle]:
        if frame is None or frame.ndim != 3:
            raise ValueError('BGR frame is required')
        height, width = frame.shape[:2]
        frame_area = float(width * height)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 160)
        edges = cv2.morphologyEx(
            edges,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
        )
        contours, _hierarchy = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_ratio = area / frame_area
            if not self.minimum_area_ratio <= area_ratio <= self.maximum_area_ratio:
                continue
            perimeter = cv2.arcLength(contour, True)
            polygon = cv2.approxPolyDP(
                contour,
                self.polygon_epsilon_ratio * perimeter,
                True,
            )
            if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                continue
            x, y, box_width, box_height = cv2.boundingRect(polygon)
            if min(box_width, box_height) <= 0:
                continue
            aspect = max(box_width, box_height) / min(box_width, box_height)
            if not self.minimum_aspect_ratio <= aspect <= self.maximum_aspect_ratio:
                continue
            rectangularity = min(1.0, area / float(box_width * box_height))
            confidence = max(0.01, min(1.0, rectangularity))
            candidates.append((area, x, y, box_width, box_height, confidence))
        if self.blue_fallback_enabled:
            for candidate in self._blue_candidates(frame, frame_area):
                duplicate_index = next(
                    (
                        index
                        for index, existing in enumerate(candidates)
                        if self._overlap_ratio(candidate, existing) >= 0.60
                    ),
                    None,
                )
                if duplicate_index is None:
                    candidates.append(candidate)
                elif candidate[5] > candidates[duplicate_index][5]:
                    candidates[duplicate_index] = candidate
        margin = self.border_margin_px
        candidates = [
            candidate
            for candidate in candidates
            if candidate[1] >= margin
            and candidate[2] >= margin
            and candidate[1] + candidate[3] <= width - margin
            and candidate[2] + candidate[4] <= height - margin
            and candidate[3] / width <= self.maximum_width_ratio
            and candidate[4] / height <= self.maximum_height_ratio
        ]
        candidates.sort(key=lambda value: (-value[0], value[1], value[2]))
        return [
            PanelRectangle(index, x, y, box_width, box_height, confidence)
            for index, (_area, x, y, box_width, box_height, confidence)
            in enumerate(candidates[:self.maximum_panels], 1)
        ]
