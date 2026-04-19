#!/usr/bin/env python3
# ── rover/test_drive_straight.py ─────────────────────────────────────────────
# Straight-line drive test with live MJPEG stream.
#
# Behaviour:
#   CLEAR  (> 60 cm)  → drive forward full speed
#   SLOW   (25–60 cm) → drive forward half speed
#   STOP   (< 25 cm)  → halt and wait until path clears
#   NO TURNING — validates straight movement + all sensors first.
#
# Stream: http://10.10.10.90:8080
# Usage:  sudo ../venv/bin/python3 test_drive_straight.py
# Ctrl+C to stop safely.

import cv2
import sys
import os
import time
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController
from vision.detector   import ObstacleDetector
from vision.ultrasonic import UltrasonicSensor
from config import (
    LOOP_HZ,
    ULTRASONIC_HARD_STOP_CM,
    ULTRASONIC_WARN_CM,
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
    DRIVE_SPEED_FULL,
    DRIVE_SPEED_SLOW,
)

STREAM_PORT = 8080

shared = {
    'distance': None,
    'scores':   None,
    'decision': 'INIT',
    'speed':    0,
}
shared_lock  = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True


# ── Annotation ────────────────────────────────────────────────────────────────

def _annotate(frame):
    with shared_lock:
        scores   = shared['scores'] or {}
        dist     = shared['distance']
        decision = shared['decision']
        speed    = shared['speed']

    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2

    col = (0, 0, 255) if decision == 'STOP' else (0, 165, 255) if decision == 'SLOW' else (0, 255, 0)

    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), col, 2)

    label = f"{decision}  {speed}%" if speed else decision
    cv2.putText(frame, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)

    if dist is not None:
        dist_str = f"US: {dist:.0f} cm"
        dist_col = (0, 0, 255) if dist < ULTRASONIC_HARD_STOP_CM else (
                   (0, 165, 255) if dist < ULTRASONIC_WARN_CM else (0, 255, 0))
    else:
        dist_str, dist_col = "US: TIMEOUT", (128, 128, 128)
    cv2.putText(frame, dist_str, (w - 220, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, dist_col, 2)

    y = 75
    for name, key, bar_col in [
        ("Contour", "contour", (255, 200, 0)),
        ("Motion",  "motion",  (0, 200, 255)),
        ("Color",   "color",   (200, 0, 255)),
        ("Edge",    "edge",    (200, 200, 200)),
    ]:
        val = scores.get(key, 0.0)
        cv2.putText(frame, f"{name}: {val:.2f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bar_col, 1)
        cv2.rectangle(frame, (140, y - 12), (140 + int(val * 150), y), bar_col, -1)
        y += 25

    combined = scores.get('combined', 0.0)
    cv2.putText(frame, f"Combined: {combined:.2f} (>{CAMERA_OBSTACLE_SCORE_THRESHOLD})",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    # Distance bar at bottom
    if dist is not None:
        bx, by, bw = 10, h - 25, w - 20
        frac = min(dist / 200.0, 1.0)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + 15), (50, 50, 50), -1)
        cv2.rectangle(frame, (bx, by), (bx + int(frac * bw), by + 15), dist_col, -1)
        for thresh, tlabel in [(ULTRASONIC_HARD_STOP_CM, "STOP"), (ULTRASONIC_WARN_CM, "SLOW")]:
            tx = bx + int((thresh / 200.0) * bw)
            cv2.line(frame, (tx, by), (tx, by + 15), (255, 255, 255), 1)
            cv2.putText(frame, tlabel, (tx - 15, by - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    return frame


# ── Camera thread ─────────────────────────────────────────────────────────────

def _camera_loop(detector):
    global latest_frame, _running
    while _running:
        try:
            raw = detector.capture_frame()
            if raw is None:
                time.sleep(0.05)
                continue
            scores, _ = detector.check()
            if scores is None:
                scores = {'contour': 0.0, 'motion': 0.0, 'color': 0.0, 'edge': 0.0, 'combined': 0.0}
            with shared_lock:
                shared['scores'] = scores
            with frame_lock:
                latest_frame = _annotate(raw.copy())
        except Exception as exc:
            print(f"[camera] {exc}")
            time.sleep(0.5)


# ── HTTP stream ───────────────────────────────────────────────────────────────

class _StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"""<!DOCTYPE html>
<html><head><title>Straight Drive Test</title>
<style>
  body{background:#1a1a1a;color:#fff;font-family:monospace;
       display:flex;flex-direction:column;align-items:center;margin-top:20px}
  h1{color:#00ff88} p{color:#aaa;margin:4px;font-size:13px}
  img{border:2px solid #00ff88;max-width:95vw}
</style></head><body>
  <h1>Straight Drive Test</h1>
  <p>Green = CLEAR full speed | Orange = SLOW | Red = STOP (waiting)</p>
  <p>No turning — straight movement + sensor validation only</p>
  <img src="/stream"/>
</body></html>""")
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while _running:
                with frame_lock:
                    if latest_frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpeg = cv2.imencode(".jpg", latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                                     jpeg.tobytes() + b"\r\n")
                except BrokenPipeError:
                    break
                time.sleep(0.05)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


# ── Drive decision ────────────────────────────────────────────────────────────

def _decide(dist, scores):
    combined       = scores['combined'] if scores else 0.0
    camera_blocked = combined > CAMERA_OBSTACLE_SCORE_THRESHOLD

    if dist is not None and dist < ULTRASONIC_HARD_STOP_CM:
        return 'STOP', 0
    if dist is not None and dist < ULTRASONIC_WARN_CM and camera_blocked:
        return 'STOP', 0
    if dist is not None and dist < ULTRASONIC_WARN_CM:
        return 'SLOW', DRIVE_SPEED_SLOW
    if camera_blocked:
        return 'SLOW', DRIVE_SPEED_SLOW
    return 'CLEAR', DRIVE_SPEED_FULL


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _running

    print("=== Straight drive test — no turning ===")
    print(f"Stream: http://10.10.10.90:{STREAM_PORT}")
    print("Ctrl+C to stop.\n")

    motors   = MotorController()
    motors.send_command("V90")

    sonar = None
    try:
        sonar = UltrasonicSensor()
        print("Ultrasonic: OK")
    except Exception as exc:
        print(f"Ultrasonic: FAILED ({exc}) — camera-only mode")

    detector = ObstacleDetector()
    print("Camera: OK\n")

    Thread(target=_camera_loop, args=(detector,), daemon=True).start()

    server = HTTPServer(("0.0.0.0", STREAM_PORT), _StreamHandler)
    Thread(target=server.serve_forever, daemon=True).start()

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    interval     = 1.0 / LOOP_HZ
    prev_decision = None

    try:
        while _running:
            dist = sonar.read_cm() if sonar else None

            with shared_lock:
                scores = shared['scores']
                shared['distance'] = dist

            decision, speed = _decide(dist, scores)

            with shared_lock:
                shared['decision'] = decision
                shared['speed']    = speed

            if decision == 'CLEAR':
                motors.drive_forward(speed)
            elif decision == 'SLOW':
                motors.drive_forward(speed)
            else:
                motors.stop()

            # Only print when decision changes to keep output readable
            if decision != prev_decision:
                dist_str  = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"
                score_str = f"{scores['combined']:.2f}" if scores else "N/A"
                print(f"{dist_str:<10} | cam={score_str} | {decision}  {speed}%")
                prev_decision = decision

            time.sleep(interval)

    finally:
        _running = False
        motors.send_command("V90")
        motors.stop()
        motors.close()
        if sonar:
            sonar.close()
        detector.close()
        server.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
