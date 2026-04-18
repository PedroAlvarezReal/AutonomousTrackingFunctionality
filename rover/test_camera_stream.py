#!/usr/bin/env python3
# ── rover/test_camera_stream.py ──────────────────────────────────────────────
# Live MJPEG stream showing all 4 detection methods + combined score.
# Open http://<rubik-pi-ip>:8080 in your browser to watch in real time.
#
# Usage:   python3 test_camera_stream.py
# Ctrl+C to stop.

import cv2
import sys
import os
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from vision.detector import ObstacleDetector
from config import (
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
)

PORT = 8080
latest_frame = None
frame_lock = Lock()


def annotate_frame(frame, scores, is_obstacle):
    """Draw danger zone, all scores, and status on the frame."""
    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2

    # Danger zone rectangle — red if obstacle, green if clear
    color = (0, 0, 255) if is_obstacle else (0, 255, 0)
    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), color, 2)

    status = "OBSTACLE" if is_obstacle else "CLEAR"
    cv2.putText(frame, f"Status: {status}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # Individual scores with colour-coded bars
    y = 65
    for name, key, col in [
        ("Contour", "contour", (255, 200, 0)),
        ("Motion", "motion", (0, 200, 255)),
        ("Color", "color", (200, 0, 255)),
        ("Edge", "edge", (200, 200, 200)),
    ]:
        val = scores.get(key, 0.0)
        bar_w = int(val * 150)
        cv2.putText(frame, f"{name}: {val:.2f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.rectangle(frame, (160, y - 12), (160 + bar_w, y), col, -1)
        y += 25

    # Combined score
    combined = scores.get('combined', 0.0)
    comb_col = (0, 0, 255) if is_obstacle else (0, 255, 0)
    cv2.putText(frame, f"COMBINED: {combined:.2f} (thresh: {CAMERA_OBSTACLE_SCORE_THRESHOLD})",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, comb_col, 2)

    return frame


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>Rover Camera Stream</title>
<style>
  body { background: #1a1a1a; color: white; font-family: monospace;
         display: flex; flex-direction: column; align-items: center; margin-top: 20px; }
  h1 { color: #00ff88; }
  img { border: 2px solid #00ff88; max-width: 95vw; }
  .info { margin: 10px; color: #aaa; }
</style></head>
<body>
  <h1>Rover Camera — Multi-Method Detection</h1>
  <p class="info">Contour | Motion | Color | Edge → Combined Score</p>
  <p class="info">Green = clear | Red = obstacle</p>
  <img src="/stream" />
</body></html>"""
            self.wfile.write(html.encode())

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with frame_lock:
                    if latest_frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpeg = cv2.imencode(".jpg", latest_frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                buf = jpeg.tobytes()
                try:
                    self.wfile.write(b"--frame\r\n"
                                     b"Content-Type: image/jpeg\r\n\r\n" +
                                     buf + b"\r\n")
                except BrokenPipeError:
                    break
                time.sleep(0.05)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def camera_loop():
    global latest_frame
    detector = ObstacleDetector()
    try:
        while True:
            raw = detector.capture_frame()
            if raw is None:
                continue
            scores, is_obstacle = detector.check()
            if scores is None:
                scores = {'contour': 0, 'motion': 0, 'color': 0, 'edge': 0, 'combined': 0}
            annotated = annotate_frame(raw.copy(), scores, is_obstacle)
            with frame_lock:
                latest_frame = annotated
    except KeyboardInterrupt:
        pass
    finally:
        detector.close()


def main():
    print(f"=== Rover Camera Stream — Multi-Method Detection ===")
    print(f"Open http://10.10.10.90:{PORT} in your Mac browser")
    print(f"Ctrl+C to stop.\n")

    cam_thread = Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping stream.")
        server.shutdown()


if __name__ == "__main__":
    main()
