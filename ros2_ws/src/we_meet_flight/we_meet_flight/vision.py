"""Downward-camera dark quadrilateral detector; this is not a semantic model."""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    cx: float
    cy: float
    area: float
    score: float
    corners: tuple


@dataclass(frozen=True)
class Observation:
    received_s: float
    sequence: int
    visible: bool
    ex: float = 0.0
    ey: float = 0.0
    area: float = 0.0
    score: float = 0.0
    track_id: int = 0


class PanelDetector:
    def __init__(self, config):
        self.c = config

    def detect(self, bgr):
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError('expected uint8 BGR image')
        h, w = bgr.shape[:2]
        if min(h, w) < 32:
            return []
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        smooth = cv2.GaussianBlur(gray, (5, 5), 0)
        # Dark panel on a lighter ground. Morphology joins small cell dividers.
        _, mask = cv2.threshold(smooth, self.c.panel_dark_threshold, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            fraction = area / (h * w)
            if not self.c.min_panel_area_fraction <= fraction <= self.c.max_panel_area_fraction:
                continue
            quad = cv2.approxPolyDP(contour, 0.025 * cv2.arcLength(contour, True), True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            pts = quad.reshape(4, 2)
            # A clipped rectangle does not provide a trustworthy panel centre.
            if np.any(pts[:, 0] < 3) or np.any(pts[:, 0] > w-4) or np.any(pts[:, 1] < 3) or np.any(pts[:, 1] > h-4):
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(quad)
            if min(rw, rh) < 8:
                continue
            aspect = max(rw, rh) / min(rw, rh)
            rectangularity = area / (rw * rh)
            if not self.c.min_panel_aspect <= aspect <= self.c.max_panel_aspect or rectangularity < 0.72:
                continue
            polygon = np.zeros((h, w), np.uint8)
            cv2.fillConvexPoly(polygon, pts, 255)
            dark = float(np.mean(gray[polygon != 0] < self.c.panel_dark_threshold))
            if dark < self.c.min_panel_dark_fraction:
                continue
            moments = cv2.moments(quad)
            if moments['m00'] <= 0:
                continue
            candidates.append(Detection(moments['m10']/moments['m00']/w,
                                        moments['m01']/moments['m00']/h,
                                        fraction, min(1.0, rectangularity)*dark,
                                        tuple(tuple(int(v) for v in pt) for pt in pts)))
        return sorted(candidates, key=lambda d: d.score, reverse=True)


class TargetTracker:
    """Acquire distinct frames, then retain one target; never jump to another."""
    def __init__(self, config):
        self.c = config
        self.last = None
        self.last_seen = None
        self.count = 0
        self.locked = False
        self.sequence = 0
        self.last_stamp = None

    def update(self, candidates, now_s, frame_stamp):
        # Duplicate camera frames cannot acquire a target or refresh its age.
        if self.last_stamp is not None and frame_stamp <= self.last_stamp:
            return None
        self.last_stamp = frame_stamp
        self.sequence += 1
        if self.last_seen is not None and now_s - self.last_seen > self.c.target_loss_s:
            if self.locked:
                return Observation(now_s, self.sequence, False, track_id=1)
            self.last, self.count = None, 0
        selected = None
        if self.last is None:
            selected = candidates[0] if candidates else None
        else:
            matches = [d for d in candidates
                       if math.hypot(d.cx-self.last.cx, d.cy-self.last.cy) <= self.c.association_radius_norm
                       and 0.45 <= d.area/self.last.area <= 2.2]
            if matches:
                selected = min(matches, key=lambda d: math.hypot(d.cx-self.last.cx, d.cy-self.last.cy))
            elif not self.locked:
                self.last, self.count = None, 0
                selected = candidates[0] if candidates else None
        if selected is None:
            if not self.locked:
                self.count = 0
            return Observation(now_s, self.sequence, False, track_id=1 if self.locked else 0)
        self.last = selected
        self.last_seen = now_s
        self.count += 1
        self.locked = self.locked or self.count >= self.c.acquisition_frames
        return Observation(now_s, self.sequence, self.locked, selected.cx-0.5,
                           selected.cy-0.5, selected.area, selected.score, 1 if self.locked else 0)
