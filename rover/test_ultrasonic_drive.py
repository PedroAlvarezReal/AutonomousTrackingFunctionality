#!/usr/bin/env python3
# ── rover/test_ultrasonic_drive.py ───────────────────────────────────────────
# Ultrasonic-only drive test.
# Motor decisions come from ultrasonic distance alone — no camera detection.
# Camera runs as a plain live stream (no scoring, no obstacle logic).
#
#   > 60 cm  → FORWARD full speed
#   25-60 cm → FORWARD slow
#   < 25 cm  → STOP
#
# Stream: http://10.10.10.90:8080
# Usage:  sudo /home/ubuntu/AutonomousTrackingFunctionality/venv/bin/python3 test_ultrasonic_drive.py

import cv2
import sys
import os
import time
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController
from vision.ultrasonic import UltrasonicSensor
from config import (
    LOOP_HZ,
    ULTRASONIC_HARD_STOP_CM,
    ULTRASONIC_WARN_CM,
    CAMERA_INDEX,
    DRIVE_SPEED_FULL,
    DRIVE_SPEED_SLOW,
)

STREAM_PORT = 8080

shared = {'distance': None, 'decision': 'INIT', 'speed': 0}
shared_lock  = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True


# ── Camera thread — live view only, no detection ──────────────────────────────

def _camera_loop(cap):
    global latest_frame, _running
    while _running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        with shared_lock:
            dist     = shared['distance']
            decision = shared['decision']
            speed    = shared['speed']

        h, w = frame.shape[:2]

        # Decision banner
        col = (0, 0, 255) if decision == 'STOP' else (0, 165, 255) if decision == 'SLOW' else (0, 255, 0)
        label = f"{decision}  {speed}%" if speed else decision
        cv2.putText(frame, label, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)

        # Distance top-right
        if dist is not None:
            dist_str = f"{dist:.0f} cm"
            dist_col = (0, 0, 255) if dist < ULTRASONIC_HARD_STOP_CM else (
                       (0, 165, 255) if dist < ULTRASONIC_WARN_CM else (0, 255, 0))
        else:
            dist_str, dist_col = "TIMEOUT", (128, 128, 128)
        cv2.putText(frame, dist_str, (w - 180, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, dist_col, 2)

        # Distance bar at bottom
        bar_x, bar_y, bar_w = 10, h - 25, w - 20
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 15), (50, 50, 50), -1)
        if dist is not None:
            frac = min(dist / 200.0, 1.0)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(frac * bar_w), bar_y + 15), dist_col, -1)
            for thresh, tlabel in [(ULTRASONIC_HARD_STOP_CM, "STOP"), (ULTRASONIC_WARN_CM, "SLOW")]:
                tx = bar_x + int((thresh / 200.0) * bar_w)
                cv2.line(frame, (tx, bar_y), (tx, bar_y + 15), (255, 255, 255), 1)
                cv2.putText(frame, tlabel, (tx - 15, bar_y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

        with frame_lock:
            latest_frame = frame


# ── HTTP stream ───────────────────────────────────────────────────────────────

class _StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"""<!DOCTYPE html>
<html><head><title>Ultrasonic Drive Test</title>
<style>
  body{background:#1a1a1a;color:#fff;font-family:monospace;
       display:flex;flex-direction:column;align-items:center;margin-top:20px}
  h1{color:#00ff88} p{color:#aaa;margin:4px;font-size:13px}
  img{border:2px solid #00ff88;max-width:95vw}
</style></head><body>
  <h1>Ultrasonic Drive Test</h1>
  <p>Green = FORWARD | Orange = SLOW | Red = STOP</p>
  <p>Camera is live view only - no detection running</p>
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


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _running

    print("=== Ultrasonic-only drive test ===")
    print(f"Stream: http://10.10.10.90:{STREAM_PORT}")
    print("Ctrl+C to stop.\n")

    motors = MotorController()
    motors.send_command("V90")

    sonar = UltrasonicSensor()
    print("Ultrasonic: OK")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Camera: FAILED — stream will be blank")
    else:
        for _ in range(5):
            cap.read()
        print("Camera: OK (live view only)\n")

    Thread(target=_camera_loop, args=(cap,), daemon=True).start()

    server = HTTPServer(("0.0.0.0", STREAM_PORT), _StreamHandler)
    Thread(target=server.serve_forever, daemon=True).start()

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    interval  = 1.0 / LOOP_HZ
    turn_dir  = 1   # alternates: 1 = left, -1 = right

    try:
        while _running:
            dist = sonar.read_cm()

            if dist is not None and dist < ULTRASONIC_HARD_STOP_CM:
                decision, speed = 'STOP', 0
            elif dist is not None and dist < ULTRASONIC_WARN_CM:
                decision, speed = 'SLOW', DRIVE_SPEED_SLOW
            else:
                decision, speed = 'CLEAR', DRIVE_SPEED_FULL

            with shared_lock:
                shared['distance'] = dist
                shared['decision'] = decision
                shared['speed']    = speed

            dist_str = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"

            if decision == 'STOP':
                # Back up briefly, then turn away from obstacle
                print(f"{dist_str:<10} | OBSTACLE — backing up and turning")
                motors.drive_backward()
                time.sleep(0.4)
                if turn_dir == 1:
                    motors.turn_left()
                    print("Turning LEFT")
                else:
                    motors.turn_right()
                    print("Turning RIGHT")
                time.sleep(0.6)
                motors.stop()
                turn_dir *= -1   # alternate direction each time
                continue         # re-evaluate immediately

            elif decision == 'SLOW':
                motors.drive_forward(speed)
                print(f"{dist_str:<10} | SLOW")
            else:
                motors.drive_forward(speed)
                print(f"{dist_str:<10} | FORWARD")

            time.sleep(interval)

    finally:
        _running = False
        motors.send_command("V90")
        motors.stop()
        motors.close()
        sonar.close()
        cap.release()
        server.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
