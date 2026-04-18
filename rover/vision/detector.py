# ── rover/vision/detector.py ─────────────────────────────────────────────────
# Multi-method obstacle detection using OpenCV.
#
# Instead of relying on edge density alone (which fails in noisy environments
# like a room full of tables, chairs, people), we combine FOUR signals:
#
#   1. CONTOUR DETECTION — finds large solid shapes (boxes, walls, furniture)
#      by looking for big blobs in the danger zone.  Works on plain objects
#      that edge detection misses.
#
#   2. FRAME DIFFERENCING — detects motion.  If something is moving toward
#      the rover (person, another robot), flag it immediately.
#
#   3. COLOUR ANOMALY — the floor is usually one consistent colour.  Anything
#      that looks different from the learned floor baseline = obstacle.
#
#   4. EDGE DENSITY — kept as a minor signal.  Useful for textured obstacles
#      but no longer the primary method.
#
# The final obstacle score (0.0–1.0) is a weighted combo of all four.
# SafetyMonitor (safety.py) fuses this with ultrasonic distance.

import cv2
import numpy as np

from config import (
    CAMERA_INDEX,
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
    CAMERA_CONTOUR_MIN_AREA_PCT,
    CAMERA_MOTION_THRESHOLD,
    CAMERA_COLOR_DIFF_THRESHOLD,
)


class ObstacleDetector:
    """Multi-method obstacle detection for the rover danger zone."""

    def __init__(self) -> None:
        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera at index {CAMERA_INDEX}. "
                "Check that the USB webcam is plugged in and /dev/video0 exists."
            )

        # Warm up — first frames are often garbage
        for _ in range(5):
            self._cap.read()

        self._prev_gray = None
        self._floor_baseline = None
        self._frame_count = 0
        self._bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=60, varThreshold=40, detectShadows=False
        )

    # ── public API ────────────────────────────────────────────────────────────

    def check(self) -> tuple[dict | None, bool]:
        """Capture one frame and return (scores_dict, obstacle_detected).

        scores_dict: {
            'contour': 0.0–1.0,   large shapes in danger zone
            'motion':  0.0–1.0,   frame-to-frame change
            'color':   0.0–1.0,   colour anomaly vs floor
            'edge':    0.0–1.0,   classic edge density (minor weight)
            'combined':0.0–1.0,   weighted final score
        }

        Returns (None, True) on capture failure — fail safe.
        """
        ret, frame = self._cap.read()
        if not ret:
            return None, True

        region = self._crop_danger_zone(frame)
        self._frame_count += 1

        contour_score = self._contour_score(region)
        motion_score = self._motion_score(region)
        color_score = self._color_anomaly_score(region)
        edge_score = self._edge_density(region)

        # Weighted combination
        combined = (
            0.35 * contour_score +
            0.20 * motion_score +
            0.30 * color_score +
            0.15 * edge_score
        )

        scores = {
            'contour': contour_score,
            'motion': motion_score,
            'color': color_score,
            'edge': edge_score,
            'combined': combined,
        }

        return scores, combined > CAMERA_OBSTACLE_SCORE_THRESHOLD

    def capture_frame(self) -> np.ndarray | None:
        """Return the raw BGR frame, or None on failure."""
        ret, frame = self._cap.read()
        return frame if ret else None

    def get_danger_zone(self, frame: np.ndarray) -> tuple[np.ndarray, int, int, int, int]:
        """Return (cropped_region, x0, y0, x1, y1) for overlay drawing."""
        h, w = frame.shape[:2]
        rh = int(h * CAMERA_REGION_HEIGHT_PCT)
        rw = int(w * CAMERA_REGION_WIDTH_PCT)
        y0 = h - rh
        x0 = (w - rw) // 2
        return frame[y0:h, x0:x0 + rw], x0, y0, x0 + rw, h

    def close(self) -> None:
        self._cap.release()

    # ── detection methods ─────────────────────────────────────────────────────

    def _crop_danger_zone(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        rh = int(h * CAMERA_REGION_HEIGHT_PCT)
        rw = int(w * CAMERA_REGION_WIDTH_PCT)
        y0 = h - rh
        x0 = (w - rw) // 2
        return frame[y0:h, x0:x0 + rw]

    def _contour_score(self, region: np.ndarray) -> float:
        """Detect large solid shapes — boxes, walls, chairs, people.

        Uses adaptive thresholding + morphological closing to find blobs,
        then scores by what fraction of the danger zone they fill.
        """
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # Adaptive threshold handles varying lighting
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 5
        )

        # Merge nearby blobs
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0

        total_area = region.shape[0] * region.shape[1]
        min_area = total_area * CAMERA_CONTOUR_MIN_AREA_PCT

        big = [c for c in contours if cv2.contourArea(c) > min_area]
        if not big:
            return 0.0

        largest_area = cv2.contourArea(max(big, key=cv2.contourArea))
        return min(1.0, largest_area / (total_area * 0.40))

    def _motion_score(self, region: np.ndarray) -> float:
        """Detect movement via background subtraction + frame differencing.

        Flags people walking, rolling objects, anything that moves.
        """
        fg_mask = self._bg_subtractor.apply(region)

        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        if self._prev_gray is None or self._prev_gray.shape != gray.shape:
            self._prev_gray = gray
            return 0.0

        diff = cv2.absdiff(self._prev_gray, gray)
        self._prev_gray = gray

        _, thresh = cv2.threshold(diff, CAMERA_MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)
        combined = cv2.bitwise_or(fg_mask, thresh)

        motion_ratio = np.count_nonzero(combined) / combined.size
        return min(1.0, motion_ratio / 0.15)

    def _color_anomaly_score(self, region: np.ndarray) -> float:
        """Detect obstacles by colour difference from the floor.

        Learns the floor colour from the first 10 frames, then flags
        anything that looks different.  Catches plain-coloured obstacles
        that edge detection would miss entirely.
        """
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

        # Bottom 30% of the region = floor reference
        h = region.shape[0]
        floor_strip = hsv[int(h * 0.7):, :]

        # Learn floor baseline from first 10 frames
        if self._frame_count <= 10:
            current_mean = np.mean(floor_strip, axis=(0, 1))
            if self._floor_baseline is None:
                self._floor_baseline = current_mean
            else:
                self._floor_baseline = 0.8 * self._floor_baseline + 0.2 * current_mean
            return 0.0

        if self._floor_baseline is None:
            return 0.0

        diff = np.abs(hsv.astype(float) - self._floor_baseline)
        # Hue matters most, then saturation, then brightness
        weighted = diff[:, :, 0] * 2.0 + diff[:, :, 1] * 1.0 + diff[:, :, 2] * 0.5

        anomaly_ratio = np.count_nonzero(weighted > CAMERA_COLOR_DIFF_THRESHOLD) / weighted.size
        return min(1.0, anomaly_ratio / 0.20)

    def _edge_density(self, region: np.ndarray) -> float:
        """Classic Canny edge density — kept as a minor signal."""
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, threshold1=50, threshold2=150)
        return float(np.count_nonzero(edges)) / edges.size
