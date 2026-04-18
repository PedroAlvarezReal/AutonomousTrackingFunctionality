# ── rover/vision/detector.py ─────────────────────────────────────────────────
# Obstacle detection using OpenCV edge density on the lower-centre frame region.
#
# Why lower-centre?
#   The rover drives forward and its camera is mounted at the front.
#   Objects in the bottom 40 % of the frame are close and directly in the path.
#   The centre 50 % horizontally cuts out the wheels / frame edges that would
#   permanently inflate the edge count.
#
# Why edge density and not a classifier?
#   Edge density is deterministic, needs no training data, and runs at >30 fps
#   on a Qualcomm QCS6490.  False positives (stopping unnecessarily) are
#   acceptable; false negatives (not stopping) are not.  The ultrasonic layer
#   is the hard backstop for any camera miss.

import cv2
import numpy as np

from config import (
    CAMERA_INDEX,
    CAMERA_EDGE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
)


class ObstacleDetector:
    """Detects obstacles by measuring Canny edge density in the danger zone."""

    def __init__(self) -> None:
        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera at index {CAMERA_INDEX}. "
                "Check that the USB webcam is plugged in and /dev/video0 exists."
            )

    # ── public API ────────────────────────────────────────────────────────────

    def check(self) -> tuple[float | None, bool]:
        """Capture one frame and return (edge_density, obstacle_detected).

        edge_density    : 0.0–1.0 (fraction of pixels that are edges)
        obstacle_detected: True when density > CAMERA_EDGE_THRESHOLD

        Returns (None, True) on capture failure — caller treats as obstacle.
        """
        ret, frame = self._cap.read()
        if not ret:
            return None, True  # fail safe

        region  = self._crop_danger_zone(frame)
        density = self._edge_density(region)
        return density, density > CAMERA_EDGE_THRESHOLD

    def capture_frame(self) -> np.ndarray | None:
        """Return the raw BGR frame, or None on failure."""
        ret, frame = self._cap.read()
        return frame if ret else None

    def close(self) -> None:
        self._cap.release()

    # ── internal helpers ──────────────────────────────────────────────────────

    def _crop_danger_zone(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]

        region_h = int(h * CAMERA_REGION_HEIGHT_PCT)
        region_w = int(w * CAMERA_REGION_WIDTH_PCT)

        y0 = h - region_h          # bottom N% of the frame
        x0 = (w - region_w) // 2   # centred horizontally

        return frame[y0:h, x0 : x0 + region_w]

    def _edge_density(self, region: np.ndarray) -> float:
        gray    = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # thresholds 50/150 are the classic Canny 1:3 ratio — adjust via
        # CAMERA_EDGE_THRESHOLD in config.py rather than touching these
        edges   = cv2.Canny(blurred, threshold1=50, threshold2=150)
        return float(np.count_nonzero(edges)) / edges.size
