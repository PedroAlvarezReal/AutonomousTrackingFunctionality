#!/usr/bin/env python3
# ── rover/test_camera.py ─────────────────────────────────────────────────────
# Captures 5 frames, saves them as JPEGs, and prints the edge density for each.
# Run this THIRD, after the ultrasonic test passes.
#
# Usage:   python3 test_camera.py     (no sudo needed — just OpenCV + USB)
#
# Output files: test_frame_01.jpg … test_frame_05.jpg  (in the current dir)
# Open them to visually confirm the camera sees what you expect.
#
# The highlighted danger zone (lower-centre crop) is drawn as a green rectangle
# so you can eyeball whether it's covering the right area.  Adjust
# CAMERA_REGION_HEIGHT_PCT / CAMERA_REGION_WIDTH_PCT in config.py if not.
#
# Edge density interpretation:
#   < 0.05  → very clean / empty scene
#   0.05–0.15 → moderate texture (carpet, grass, patterned floor)
#   > 0.15  → lots of edges — likely an obstacle at the current threshold

import cv2
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from vision.detector import ObstacleDetector
from config import (
    CAMERA_EDGE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
)

FRAMES_TO_CAPTURE = 5


def _draw_danger_zone(frame):
    """Overlay the danger-zone rectangle on the frame for visual verification."""
    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2
    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), (0, 255, 0), 2)
    cv2.putText(frame, "danger zone", (x0 + 4, y0 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame


def main() -> None:
    print("=== Camera / OpenCV test ===")
    print(f"Capturing {FRAMES_TO_CAPTURE} frames — saving to current directory.\n")

    detector = ObstacleDetector()
    try:
        for i in range(1, FRAMES_TO_CAPTURE + 1):
            # get raw frame for saving
            raw = detector.capture_frame()
            if raw is None:
                print(f"Frame {i}: FAILED — could not read from camera")
                continue

            # get edge analysis (reads a second frame — consecutive, same scene)
            density, is_obstacle = detector.check()

            filename = f"test_frame_{i:02d}.jpg"
            annotated = _draw_danger_zone(raw.copy())
            cv2.imwrite(filename, annotated)

            flag = "OBSTACLE" if is_obstacle else "clear"
            print(
                f"Frame {i:02d}: saved '{filename}' | "
                f"edge density = {density:.4f} | "
                f"threshold = {CAMERA_EDGE_THRESHOLD} | "
                f"result = {flag}"
            )
    finally:
        detector.close()
        print("\nCamera released.")
        print("\nTip: if density is always near 0, check that the lens cap is off.")
        print(      "     if density is always above threshold, lower CAMERA_EDGE_THRESHOLD in config.py.")


if __name__ == "__main__":
    main()
