#!/usr/bin/env python3
# ── rover/test_ultrasonic_drive.py ───────────────────────────────────────────
# Ultrasonic-only drive test with continuous servo sweep.
# Servo sweeps left/center/right while driving so obstacles are seen early.
# Camera is plain live stream — no detection logic.
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

# Sweep angles: left=45, center=90, right=135
# (not full 0/180 — keeps sensing relevant to the direction of travel)
SWEEP_ANGLES = [45, 90, 135]
SETTLE_S     = 0.15   # seconds for servo to reach position + sensor to stabilise

shared = {
    'readings':  {45: None, 90: None, 135: None},  # angle → distance
    'min_dist':  None,   # closest reading across all angles
    'servo_angle': 90,
    'decision':  'INIT',
    'speed':     0,
}
shared_lock  = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True
_sweep_pause = False   # drive loop sets True during backup/turn so sweep waits


# ── Sweep thread — continuously reads left/center/right ──────────────────────

def _sweep_loop(motors, sonar):
    global _running, _sweep_pause
    idx = 0
    while _running:
        if _sweep_pause:
            time.sleep(0.05)
            continue

        angle = SWEEP_ANGLES[idx % len(SWEEP_ANGLES)]
        motors.send_command(f"V{angle}")
        time.sleep(SETTLE_S)

        if _sweep_pause:   # check again after settle
            idx += 1
            continue

        dist = sonar.read_cm()

        with shared_lock:
            shared['readings'][angle] = dist
            shared['servo_angle']     = angle
            # min_dist = closest valid reading (ignore None / timeouts)
            valid = [d for d in shared['readings'].values() if d is not None]
            shared['min_dist'] = min(valid) if valid else None

        idx += 1


# ── Camera thread — live view only ───────────────────────────────────────────

def _camera_loop(cap):
    global latest_frame, _running
    while _running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        with shared_lock:
            readings  = dict(shared['readings'])
            angle     = shared['servo_angle']
            min_dist  = shared['min_dist']
            decision  = shared['decision']
            speed     = shared['speed']

        h, w = frame.shape[:2]

        col = (0, 0, 255) if decision == 'STOP' else (0, 165, 255) if decision == 'SLOW' else (0, 255, 0)
        label = f"{decision}  {speed}%" if speed else decision
        cv2.putText(frame, label, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)

        # Per-angle readings on the right
        y = 35
        for a in SWEEP_ANGLES:
            d = readings.get(a)
            tag = f"{'L' if a==45 else 'C' if a==90 else 'R'} ({a}deg): "
            tag += f"{d:.0f}cm" if d is not None else "N/A"
            a_col = (200, 200, 0) if a == angle else (150, 150, 150)
            cv2.putText(frame, tag, (w - 200, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, a_col, 1)
            y += 22

        # Min distance bar at bottom
        bar_x, bar_y, bar_w = 10, h - 25, w - 20
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 15), (50, 50, 50), -1)
        if min_dist is not None:
            dist_col = (0, 0, 255) if min_dist < ULTRASONIC_HARD_STOP_CM else (
                       (0, 165, 255) if min_dist < ULTRASONIC_WARN_CM else (0, 255, 0))
            frac = min(min_dist / 200.0, 1.0)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(frac * bar_w), bar_y + 15), dist_col, -1)
            cv2.putText(frame, f"min:{min_dist:.0f}cm", (bar_x + 5, bar_y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, dist_col, 1)
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
  <h1>Ultrasonic Sweep Drive</h1>
  <p>Green = FORWARD | Orange = SLOW | Red = STOP+turn</p>
  <p>L/C/R = left(45)/center(90)/right(135) sweep readings</p>
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


# ── Avoidance: pick best turn direction from sweep readings ───────────────────

def _best_turn(readings):
    """Return 'left' or 'right' based on which side has more clearance."""
    left  = readings.get(45)  or 0
    right = readings.get(135) or 0
    return 'left' if left >= right else 'right'


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _running, _sweep_pause

    print("=== Ultrasonic sweep drive test ===")
    print(f"Stream: http://10.10.10.90:{STREAM_PORT}")
    print("Ctrl+C to stop.\n")

    motors = MotorController()
    motors.send_command("V90")

    sonar = UltrasonicSensor()
    print("Ultrasonic: OK")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if cap.isOpened():
        for _ in range(5):
            cap.read()
        print("Camera: OK (live view only)")
    else:
        print("Camera: not found — stream blank")

    Thread(target=_sweep_loop,  args=(motors, sonar), daemon=True).start()
    Thread(target=_camera_loop, args=(cap,),           daemon=True).start()

    server = HTTPServer(("0.0.0.0", STREAM_PORT), _StreamHandler)
    Thread(target=server.serve_forever, daemon=True).start()
    print()

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    interval = 1.0 / LOOP_HZ

    try:
        while _running:
            with shared_lock:
                min_dist = shared['min_dist']
                readings = dict(shared['readings'])

            if min_dist is not None and min_dist < ULTRASONIC_HARD_STOP_CM:
                decision, speed = 'STOP', 0
            elif min_dist is not None and min_dist < ULTRASONIC_WARN_CM:
                decision, speed = 'SLOW', DRIVE_SPEED_SLOW
            else:
                decision, speed = 'CLEAR', DRIVE_SPEED_FULL

            with shared_lock:
                shared['decision'] = decision
                shared['speed']    = speed

            if decision == 'STOP':
                _sweep_pause = True
                turn = _best_turn(readings)
                print(f"min={min_dist:.1f}cm | OBSTACLE — back up + turn {turn.upper()}")

                motors.drive_backward()
                time.sleep(0.4)
                if turn == 'left':
                    motors.turn_left()
                else:
                    motors.turn_right()
                time.sleep(0.6)
                motors.stop()

                # Re-centre servo and resume sweep
                motors.send_command("V90")
                time.sleep(0.2)
                # Clear stale readings so old values don't re-trigger immediately
                with shared_lock:
                    shared['readings'] = {45: None, 90: None, 135: None}
                    shared['min_dist'] = None
                _sweep_pause = False
                continue

            elif decision == 'SLOW':
                motors.drive_forward(speed)
                print(f"min={min_dist:.1f}cm | SLOW")
            else:
                motors.drive_forward(speed)

            time.sleep(interval)

    finally:
        _running = False
        _sweep_pause = False
        motors.send_command("V90")
        motors.stop()
        motors.close()
        sonar.close()
        cap.release()
        server.shutdown()
        print("\nStopped.")


if __name__ == "__main__":
    main()
