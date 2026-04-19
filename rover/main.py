#!/usr/bin/env python3
# ── rover/main.py ────────────────────────────────────────────────────────────
# Autonomous drive loop with live MJPEG stream for monitoring.
#
# Strategy:
#   1. Drive forward — CLEAR at full speed, SLOW in warn zone.
#   2. STOP when obstacle < HARD_STOP_CM or camera + ultrasonic both trigger.
#   3. Servo sweep to find clearest direction, turn that way, resume.
#
# Stream: http://10.10.10.90:8080  (camera + scores + distance + decision)
#
# Usage:   sudo python3 main.py        (needs GPIO for ultrasonic)
# Ctrl+C / SIGTERM stops motors and releases all resources.

import cv2
import sys
import os
import time
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller  import MotorController
from vision.detector    import ObstacleDetector
from vision.ultrasonic  import UltrasonicSensor
from vision.servo_sweep import ServoSweep
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

STREAM_PORT   = 8080
TURN_DURATION = 0.8   # seconds per avoidance turn

# ── Shared state (camera thread → drive loop → stream) ───────────────────────
shared = {
    'scores':      None,
    'distance':    None,
    'servo_angle': 90,
    'decision':    'INIT',
    'speed':       0,
}
shared_lock = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True   # set False by signal handler to stop all threads


# ── Annotation ────────────────────────────────────────────────────────────────

def _annotate(frame):
    """Draw danger zone, scores, distance bar, and decision onto frame."""
    with shared_lock:
        scores    = shared['scores'] or {}
        dist      = shared['distance']
        angle     = shared['servo_angle']
        decision  = shared['decision']
        speed     = shared['speed']

    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2

    if decision == 'STOP':
        col = (0, 0, 255)
    elif decision == 'SLOW':
        col = (0, 165, 255)
    else:
        col = (0, 255, 0)

    # Danger zone border
    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), col, 2)

    # Decision banner top-left
    label = f"{decision}  {speed}%" if speed else decision
    cv2.putText(frame, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)

    # Ultrasonic top-right
    if dist is not None:
        dist_str = f"US: {dist:.0f} cm"
        dist_col = (0, 0, 255) if dist < ULTRASONIC_HARD_STOP_CM else (
                   (0, 165, 255) if dist < ULTRASONIC_WARN_CM else (0, 255, 0))
    else:
        dist_str = "US: TIMEOUT"
        dist_col = (128, 128, 128)
    cv2.putText(frame, dist_str,           (w - 220, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, dist_col, 2)
    cv2.putText(frame, f"Servo: {angle}°", (w - 220, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    # Individual score bars
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

    # Distance bar bottom
    if dist is not None:
        bar_x, bar_y = 10, h - 25
        bar_w_max    = w - 20
        frac         = min(dist / 200.0, 1.0)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w_max, bar_y + 15), (50, 50, 50), -1)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(frac * bar_w_max), bar_y + 15), dist_col, -1)
        for thresh, tlabel in [(ULTRASONIC_HARD_STOP_CM, "STOP"), (ULTRASONIC_WARN_CM, "WARN")]:
            tx = bar_x + int((thresh / 200.0) * bar_w_max)
            cv2.line(frame, (tx, bar_y), (tx, bar_y + 15), (255, 255, 255), 1)
            cv2.putText(frame, tlabel, (tx - 15, bar_y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

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

            annotated = _annotate(raw.copy())
            with frame_lock:
                latest_frame = annotated

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
<html><head><title>Rover Drive Monitor</title>
<style>
  body{background:#1a1a1a;color:#fff;font-family:monospace;
       display:flex;flex-direction:column;align-items:center;margin-top:20px}
  h1{color:#00ff88}
  img{border:2px solid #00ff88;max-width:95vw}
  p{color:#aaa;margin:4px;font-size:13px}
</style></head><body>
  <h1>Rover — Autonomous Drive</h1>
  <p>Green = CLEAR (full speed) | Orange = SLOW | Red = STOP + scan</p>
  <p>Bottom bar = ultrasonic distance | Scores = camera detection</p>
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
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                        jpeg.tobytes() + b"\r\n"
                    )
                except BrokenPipeError:
                    break
                time.sleep(0.05)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


# ── Drive helpers ─────────────────────────────────────────────────────────────

def _fuse(dist, scores):
    """Return (decision, speed) from ultrasonic distance + camera scores."""
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


def _scan_and_turn(motors, sweep):
    """Servo sweep, then turn toward the clearest direction."""
    motors.stop()
    print("Scanning for best direction...")

    readings = sweep.quick_scan()
    left_cm   = readings.get(0,   0) or 0
    center_cm = readings.get(90,  0) or 0
    right_cm  = readings.get(180, 0) or 0
    print(f"  Left={left_cm} cm  Center={center_cm} cm  Right={right_cm} cm")

    with shared_lock:
        shared['servo_angle'] = 90  # back to straight after sweep

    best_angle, best_dist = sweep.best_direction(readings)

    if best_angle < 60:
        print(f"Turning LEFT (best {best_dist} cm at {best_angle}°)")
        motors.turn_left()
        time.sleep(TURN_DURATION)
    elif best_angle > 120:
        print(f"Turning RIGHT (best {best_dist} cm at {best_angle}°)")
        motors.turn_right()
        time.sleep(TURN_DURATION)
    else:
        print("All blocked — backing up and turning left")
        motors.drive_backward()
        time.sleep(0.5)
        motors.turn_left()
        time.sleep(TURN_DURATION)

    motors.stop()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _running

    print("=== Autonomous rover — drive loop + live stream ===")
    print(f"Open http://10.10.10.90:{STREAM_PORT} to monitor.")
    print("Ctrl+C to stop.\n")

    # ── Hardware init ─────────────────────────────────────────────────────────
    motors = MotorController()
    motors.send_command("V90")

    sonar = None
    sweep = None
    try:
        sonar = UltrasonicSensor()
        sweep = ServoSweep(motors, sonar)
        print("Ultrasonic + servo ready.")
    except Exception as exc:
        print(f"Sensors unavailable ({exc}) — camera-only mode.")

    detector = ObstacleDetector()
    print("Camera ready.")

    # ── Background threads ────────────────────────────────────────────────────
    cam_thread = Thread(target=_camera_loop, args=(detector,), daemon=True)
    cam_thread.start()

    server = HTTPServer(("0.0.0.0", STREAM_PORT), _StreamHandler)
    srv_thread = Thread(target=server.serve_forever, daemon=True)
    srv_thread.start()

    # ── Signal handling ───────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    interval = 1.0 / LOOP_HZ

    # ── Drive loop ────────────────────────────────────────────────────────────
    try:
        while _running:
            dist = sonar.read_cm() if sonar else None

            with shared_lock:
                scores = shared['scores']
                shared['distance'] = dist

            decision, speed = _fuse(dist, scores)

            with shared_lock:
                shared['decision'] = decision
                shared['speed']    = speed

            dist_str  = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"
            score_str = f"{scores['combined']:.2f}" if scores else "N/A"

            if decision == 'CLEAR':
                motors.drive_forward(speed)
                print(f"{dist_str:<10} | cam={score_str} | CLEAR → forward {speed}%")

            elif decision == 'SLOW':
                motors.drive_forward(speed)
                print(f"{dist_str:<10} | cam={score_str} | SLOW  → forward {speed}%")

            else:
                print(f"{dist_str:<10} | cam={score_str} | STOP  → scanning")
                if sweep:
                    _scan_and_turn(motors, sweep)
                else:
                    motors.stop()
                    time.sleep(1.0)
                continue  # re-evaluate immediately after manoeuvre

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
        print("\nRover stopped. All resources released.")


if __name__ == "__main__":
    main()
