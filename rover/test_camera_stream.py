#!/usr/bin/env python3
import cv2
import sys
import os
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock

sys.path.insert(0, os.path.dirname(__file__))

from vision.detector import ObstacleDetector
from config import (
    CAMERA_EDGE_THRESHOLD,
    CAMERA_REGION_HEIGHT_PCT,
    CAMERA_REGION_WIDTH_PCT,
)

PORT = 8080
latest_frame = None
frame_lock = Lock()

def annotate_frame(frame, density, is_obstacle):
    h, w = frame.shape[:2]
    rh = int(h * CAMERA_REGION_HEIGHT_PCT)
    rw = int(w * CAMERA_REGION_WIDTH_PCT)
    y0 = h - rh
    x0 = (w - rw) // 2
    color = (0, 0, 255) if is_obstacle else (0, 255, 0)
    cv2.rectangle(frame, (x0, y0), (x0 + rw, h), color, 2)
    status = "OBSTACLE" if is_obstacle else "CLEAR"
    cv2.putText(frame, f"Edge density: {density:.4f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Threshold: {CAMERA_EDGE_THRESHOLD}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Status: {status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    region = frame[y0:h, x0:x0 + rw]
    gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    preview_h = min(150, edges_bgr.shape[0])
    preview_w = min(200, edges_bgr.shape[1])
    preview = cv2.resize(edges_bgr, (preview_w, preview_h))
    frame[5:5 + preview_h, w - preview_w - 5:w - 5] = preview
    cv2.putText(frame, "Edge view", (w - preview_w - 5, preview_h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return frame

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            html = '<!DOCTYPE html><html><head><title>Rover Camera</title><style>body{background:#1a1a1a;color:white;font-family:monospace;display:flex;flex-direction:column;align-items:center;margin-top:20px}h1{color:#00ff88}img{border:2px solid #00ff88;max-width:95vw}</style></head><body><h1>Rover Camera Live</h1><p>Green=clear Red=obstacle</p><img src="/stream"/></body></html>'
            self.wfile.write(html.encode())
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with frame_lock:
                    if latest_frame is None:
                        continue
                    _, jpeg = cv2.imencode(".jpg", latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                buf = jpeg.tobytes()
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n")
                except BrokenPipeError:
                    break
                time.sleep(0.05)
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
            density, is_obstacle = detector.check()
            if density is None:
                density = 0.0
            annotated = annotate_frame(raw.copy(), density, is_obstacle)
            with frame_lock:
                latest_frame = annotated
    except KeyboardInterrupt:
        pass
    finally:
        detector.close()

def main():
    print(f"=== Rover Camera Stream ===")
    print(f"Open http://10.10.10.90:{PORT} in your browser")
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
