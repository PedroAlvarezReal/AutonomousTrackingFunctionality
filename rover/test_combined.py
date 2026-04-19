#!/usr/bin/env python3
# ── rover/test_combined.py ───────────────────────────────────────────────────
# Combined camera + ultrasonic + servo test.
# MJPEG stream at http://10.10.10.90:8080 showing:
#   - Camera frame with 4-method detection scores
#   - Live ultrasonic distance overlay
#   - Servo sweep angle indicator
#   - Safety decision (CLEAR / SLOW / STOP)
#
# Usage:   sudo python3 test_combined.py   (needs GPIO for ultrasonic)
# Open http://10.10.10.90:8080 on your Mac.
# Ctrl+C to stop.

import cv2
import sys
import os
import time
import serial
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from vision.detector   import ObstacleDetector
from vision.ultrasonic import UltrasonicSensor
from config import (
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
    ULTRASONIC_HARD_STOP_CM,
    ULTRASONIC_WARN_CM,
    ARDUINO_SERIAL_PORT,
    ARDUINO_BAUD_RATE,
)

PORT = 8080
latest_frame = None
frame_lock = Lock()

# Shared sensor state
sensor_data = {
    'distance': None,
    'servo_angle': 90,
    'decision': 'INIT',
}
sensor_lock = Lock()


def annotate_frame(frame, scores, is_obstacle):
    """Draw danger zone, scores, ultrasonic distance, and decision."""
    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2

    # Read current sensor data
    with sensor_lock:
        dist = sensor_data['distance']
        angle = sensor_data['servo_angle']
        decision = sensor_data['decision']

    # Pick colour based on decision
    if decision == 'STOP':
        border_col = (0, 0, 255)   # red
    elif decision == 'SLOW':
        border_col = (0, 165, 255) # orange
    else:
        border_col = (0, 255, 0)   # green

    # Danger zone rectangle
    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), border_col, 2)

    # ── Top-left: decision banner ────────────────────────────────────────
    cv2.putText(frame, decision, (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, border_col, 3)

    # ── Top-right: ultrasonic distance ───────────────────────────────────
    if dist is not None:
        dist_str = f"US: {dist:.0f} cm"
        if dist < ULTRASONIC_HARD_STOP_CM:
            dist_col = (0, 0, 255)
        elif dist < ULTRASONIC_WARN_CM:
            dist_col = (0, 165, 255)
        else:
            dist_col = (0, 255, 0)
    else:
        dist_str = "US: TIMEOUT"
        dist_col = (128, 128, 128)

    cv2.putText(frame, dist_str, (w - 220, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, dist_col, 2)
    cv2.putText(frame, f"Servo: {angle}deg", (w - 220, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    # ── Left side: individual scores with bars ───────────────────────────
    y = 75
    for name, key, col in [
        ("Contour", "contour", (255, 200, 0)),
        ("Motion",  "motion",  (0, 200, 255)),
        ("Color",   "color",   (200, 0, 255)),
        ("Edge",    "edge",    (200, 200, 200)),
    ]:
        val = scores.get(key, 0.0)
        bar_w = int(val * 150)
        cv2.putText(frame, f"{name}: {val:.2f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.rectangle(frame, (140, y - 12), (140 + bar_w, y), col, -1)
        y += 25

    # Combined score
    combined = scores.get('combined', 0.0)
    cv2.putText(frame, f"Combined: {combined:.2f} (>{CAMERA_OBSTACLE_SCORE_THRESHOLD})",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_col, 2)

    # ── Bottom: distance bar visualisation ───────────────────────────────
    if dist is not None:
        bar_max_cm = 200.0
        bar_frac = min(dist / bar_max_cm, 1.0)
        bar_x = 10
        bar_y = h - 25
        bar_w_max = w - 20
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w_max, bar_y + 15),
                      (50, 50, 50), -1)
        cv2.rectangle(frame, (bar_x, bar_y),
                      (bar_x + int(bar_frac * bar_w_max), bar_y + 15),
                      dist_col, -1)
        # Draw threshold markers
        for thresh, label in [(ULTRASONIC_HARD_STOP_CM, "STOP"),
                               (ULTRASONIC_WARN_CM, "WARN")]:
            tx = bar_x + int((thresh / bar_max_cm) * bar_w_max)
            cv2.line(frame, (tx, bar_y), (tx, bar_y + 15), (255, 255, 255), 1)
            cv2.putText(frame, label, (tx - 15, bar_y - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    return frame


def compute_decision(dist, scores):
    """Simplified fusion logic matching safety.py rules."""
    combined = scores.get('combined', 0.0) if scores else 0.0
    camera_obstacle = combined > CAMERA_OBSTACLE_SCORE_THRESHOLD

    if dist is not None and dist < ULTRASONIC_HARD_STOP_CM:
        return 'STOP'
    if dist is not None and dist < ULTRASONIC_WARN_CM and camera_obstacle:
        return 'STOP'
    if dist is not None and dist < ULTRASONIC_WARN_CM:
        return 'SLOW'
    if camera_obstacle:
        return 'SLOW'
    return 'CLEAR'


def sensor_loop(arduino_ser, ultrasonic):
    """Read ultrasonic + do periodic mini-sweeps."""
    angles = [90]  # Just forward for now; expand to sweep if desired
    idx = 0

    while True:
        try:
            angle = angles[idx % len(angles)]
            arduino_ser.write(f"V{angle}\n".encode())
            time.sleep(0.3)

            dist = ultrasonic.read_cm()

            with sensor_lock:
                sensor_data['distance'] = dist
                sensor_data['servo_angle'] = angle

            idx += 1
            time.sleep(0.1)
        except Exception as e:
            print(f"Sensor error: {e}")
            time.sleep(1)


def camera_loop(detector):
    """Capture frames, score them, annotate, and update shared frame."""
    global latest_frame

    while True:
        try:
            raw = detector.capture_frame()
            if raw is None:
                time.sleep(0.05)
                continue

            scores, is_obstacle = detector.check()
            if scores is None:
                scores = {'contour': 0, 'motion': 0, 'color': 0,
                          'edge': 0, 'combined': 0}

            # Compute decision using latest ultrasonic reading
            with sensor_lock:
                dist = sensor_data['distance']
            decision = compute_decision(dist, scores)
            with sensor_lock:
                sensor_data['decision'] = decision

            annotated = annotate_frame(raw.copy(), scores, is_obstacle)
            with frame_lock:
                latest_frame = annotated

        except Exception as e:
            print(f"Camera error: {e}")
            time.sleep(0.5)


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>Rover Combined Test</title>
<style>
  body { background: #1a1a1a; color: white; font-family: monospace;
         display: flex; flex-direction: column; align-items: center;
         margin-top: 20px; }
  h1 { color: #00ff88; }
  img { border: 2px solid #00ff88; max-width: 95vw; }
  .info { margin: 5px; color: #aaa; font-size: 14px; }
</style></head>
<body>
  <h1>Rover — Camera + Ultrasonic Combined</h1>
  <p class="info">Camera: Contour | Motion | Color | Edge → Combined</p>
  <p class="info">Ultrasonic: distance bar (bottom) | STOP &lt; 25cm | WARN &lt; 60cm</p>
  <p class="info">Green = CLEAR | Orange = SLOW | Red = STOP</p>
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

    def log_message(self, fmt, *args):
        pass


def main():
    print("=== Rover Combined Test — Camera + Ultrasonic + Servo ===")
    print(f"Open http://10.10.10.90:{PORT} in your Mac browser")
    print("Ctrl+C to stop.\n")

    # Init Arduino serial (for servo)
    print("Connecting to Arduino...")
    arduino_ser = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_BAUD_RATE, timeout=1)
    time.sleep(2)
    arduino_ser.reset_input_buffer()
    arduino_ser.write(b"V90\n")  # center servo
    print("Arduino ready.")

    # Init ultrasonic
    print("Initialising ultrasonic...")
    try:
        ultrasonic = UltrasonicSensor()
        print("Ultrasonic ready.")
    except Exception as e:
        print(f"⚠️  Ultrasonic failed: {e} — will show TIMEOUT")
        ultrasonic = None

    # Init camera
    print("Initialising camera...")
    detector = ObstacleDetector()
    print("Camera ready.\n")

    # Start sensor thread (ultrasonic + servo)
    if ultrasonic:
        sen_thread = Thread(target=sensor_loop,
                            args=(arduino_ser, ultrasonic), daemon=True)
        sen_thread.start()

    # Start camera thread
    cam_thread = Thread(target=camera_loop, args=(detector,), daemon=True)
    cam_thread.start()

    # HTTP server on main thread
    server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...")
        server.shutdown()
    finally:
        arduino_ser.write(b"V90\n")
        arduino_ser.close()
        if ultrasonic:
            ultrasonic.close()
        detector.close()
        print("Done.")


if __name__ == "__main__":
    main()
