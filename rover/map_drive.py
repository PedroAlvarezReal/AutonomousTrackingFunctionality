#!/usr/bin/env python3
# ── rover/map_drive.py ───────────────────────────────────────────────────────
# Interactive map navigation + live camera stream.
#
# Click anywhere on the map to send the rover to that location.
# Rover navigates using GPS when available and keeps the ultrasonic sensor
# pointed forward while cruising. When something gets close, it performs a
# short left/center/right scan to pick the best avoidance turn.
# Camera streams live at all times.
#
# Stream: http://10.10.10.90:8080
# Usage:  sudo /home/ubuntu/AutonomousTrackingFunctionality/venv/bin/python3 map_drive.py

import cv2, sys, os, time, signal, math, json
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(__file__))

from gps.reader import GPSReader
from gps.navigator import get_bearing, get_distance_meters
from imu.mpu9250 import MPU9250Heading
from motors.controller import MotorController
from vision.ultrasonic  import UltrasonicSensor
from config import (
    LOOP_HZ, CAMERA_INDEX,
    ULTRASONIC_HARD_STOP_CM, ULTRASONIC_WARN_CM,
    DRIVE_SPEED_FULL, DRIVE_SPEED_SLOW,
)

PORT = 8080

# ── Tunable constants ─────────────────────────────────────────────────────────
FORWARD_MPS            = 0.25   # estimated m/s at full speed — calibrate after testing
SLOW_MPS               = 0.12   # estimated m/s at slow speed
TURN_DPS               = 110.0  # estimated degrees/sec when turning in place
ARRIVE_M               = 0.35   # metres — close enough to count as arrived
HEADING_TOL            = 12     # degrees — acceptable heading error before driving
TRAIL_STEP             = 0.15   # metres — minimum distance before adding trail point
CRUISE_SERVO_ANGLE     = 90
ADAPTIVE_SWEEP_ANGLES  = [45, 90, 135]
FRONT_SAMPLE_S         = 0.12
SCAN_SETTLE_S          = 0.18
AVOID_BACKUP_S         = 0.35
AVOID_TURN_S           = 0.45
TURN_PREFERENCE_CM     = 12.0
THRESHOLD_MIN_CM       = 5
THRESHOLD_MAX_CM       = 120
THRESHOLD_GAP_CM       = 5
GPS_HEADING_MIN_MOVE_M = 0.50

# ── Shared state ──────────────────────────────────────────────────────────────
nav = {
    'x': 0.0, 'y': 0.0,           # position in metres from start
    'heading': 0.0,                # degrees: 0=North, 90=East, 180=South, 270=West
    'heading_valid': False,
    'heading_source': 'unknown',
    'imu_enabled': False,
    'lat': None,
    'lon': None,
    'target_lat': None,
    'target_lon': None,
    'target_x': None,
    'target_y': None,
    'dist_to_target': None,
    'us_dist': None,               # forward ultrasonic reading used for cruise decisions
    'us_hard_stop_cm': ULTRASONIC_HARD_STOP_CM,
    'us_warn_cm': ULTRASONIC_WARN_CM,
    'servo_angle': 90,
    'sweep': {45: None, 90: None, 135: None},
    'decision': 'IDLE',
    'trail': [],
}
nav_lock     = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True
_sweep_pause = False   # paused during backup/turns or active scans


# ── Helpers ───────────────────────────────────────────────────────────────────

class _NullMotorController:
    """No-op motor interface so the UI can start without Arduino hardware."""

    def send_command(self, *_args, **_kwargs):
        pass

    def stop(self):
        pass

    def close(self):
        pass

    def drive_backward(self):
        pass

    def turn_left(self):
        pass

    def turn_right(self):
        pass

    def drive_forward(self, _speed=0):
        pass


class _NullUltrasonicSensor:
    """Fallback sonar interface when the ultrasonic hardware is unavailable."""

    def read_cm(self):
        return None

    def close(self):
        pass

def _bearing(x1, y1, x2, y2):
    """Compass bearing from (x1,y1) to (x2,y2). 0=North, clockwise."""
    dx, dy = x2 - x1, y2 - y1
    return math.degrees(math.atan2(dx, dy)) % 360


def _heading_error(current, target):
    """Signed heading error in degrees. Negative = turn left, positive = turn right."""
    return (target - current + 180) % 360 - 180


def _heading_is_imu_source(source):
    return isinstance(source, str) and source.startswith('imu')


def _add_trail(x, y):
    trail = nav['trail']
    if trail:
        lx, ly = trail[-1]
        if math.hypot(x - lx, y - ly) < TRAIL_STEP:
            return
    trail.append((x, y))
    if len(trail) > 500:
        trail.pop(0)


def _latlon_offset_m(lat1, lon1, lat2, lon2):
    """Approximate east/north offset in metres between two GPS points."""
    mean_lat = math.radians((lat1 + lat2) / 2.0)
    north_m = (lat2 - lat1) * 111320.0
    east_m = (lon2 - lon1) * 111320.0 * math.cos(mean_lat)
    return east_m, north_m


def _sync_target_from_gps_locked():
    """Project a GPS target into the local x/y frame used by the drive loop."""
    if None in (nav['lat'], nav['lon'], nav['target_lat'], nav['target_lon']):
        return
    east_m, north_m = _latlon_offset_m(
        nav['lat'], nav['lon'],
        nav['target_lat'], nav['target_lon'],
    )
    nav['target_x'] = nav['x'] + east_m
    nav['target_y'] = nav['y'] + north_m


def _bearing_from_latlon(lat1, lon1, lat2, lon2):
    """Compass bearing from one GPS point to another."""
    east_m, north_m = _latlon_offset_m(lat1, lon1, lat2, lon2)
    return math.degrees(math.atan2(east_m, north_m)) % 360


# ── Ultrasonic helpers ────────────────────────────────────────────────────────

def _normalize_thresholds(hard_stop_cm, warn_cm):
    """Clamp live UI thresholds to a safe, ordered pair."""
    hard_stop = int(round(float(hard_stop_cm)))
    warn = int(round(float(warn_cm)))

    hard_stop = max(THRESHOLD_MIN_CM, min(THRESHOLD_MAX_CM - THRESHOLD_GAP_CM, hard_stop))
    warn = max(hard_stop + THRESHOLD_GAP_CM, min(THRESHOLD_MAX_CM, warn))
    hard_stop = min(hard_stop, warn - THRESHOLD_GAP_CM)
    return hard_stop, warn

def _clearance_value(dist):
    """Treat missing/timeout readings as blocked when comparing scan results."""
    return -1.0 if dist is None else float(dist)


def _set_servo_angle(motors, angle, settle_s=0.0):
    angle = int(max(0, min(180, angle)))
    with nav_lock:
        current = nav['servo_angle']
    if current != angle:
        motors.send_command(f"V{angle}")
        with nav_lock:
            nav['servo_angle'] = angle
    if settle_s > 0:
        time.sleep(settle_s)


def _choose_turn_direction(readings, preferred_turn=None):
    left_clear = _clearance_value(readings.get(45))
    right_clear = _clearance_value(readings.get(135))

    if preferred_turn == 'left' and left_clear >= right_clear - TURN_PREFERENCE_CM:
        return 'left'
    if preferred_turn == 'right' and right_clear >= left_clear - TURN_PREFERENCE_CM:
        return 'right'
    return 'left' if left_clear >= right_clear else 'right'


def _adaptive_scan(motors, sonar):
    """Pause cruising updates, scan left/center/right, then re-center."""
    global _sweep_pause

    _sweep_pause = True
    readings = {}
    try:
        for angle in ADAPTIVE_SWEEP_ANGLES:
            _set_servo_angle(motors, angle, SCAN_SETTLE_S)
            dist = sonar.read_cm()
            readings[angle] = dist
            with nav_lock:
                nav['sweep'][angle] = dist
                if angle == CRUISE_SERVO_ANGLE:
                    nav['us_dist'] = dist
        _set_servo_angle(motors, CRUISE_SERVO_ANGLE, SCAN_SETTLE_S)
        with nav_lock:
            nav['servo_angle'] = CRUISE_SERVO_ANGLE
        return readings
    finally:
        _sweep_pause = False


# ── Ultrasonic cruise thread ─────────────────────────────────────────────────

def _sweep_loop(motors, sonar):
    global _running, _sweep_pause
    _set_servo_angle(motors, CRUISE_SERVO_ANGLE, SCAN_SETTLE_S)
    while _running:
        if _sweep_pause:
            time.sleep(0.05)
            continue

        _set_servo_angle(motors, CRUISE_SERVO_ANGLE)
        dist = sonar.read_cm()
        with nav_lock:
            nav['sweep'][CRUISE_SERVO_ANGLE] = dist
            nav['servo_angle'] = CRUISE_SERVO_ANGLE
            nav['us_dist'] = dist
        time.sleep(FRONT_SAMPLE_S)


# ── Camera thread ─────────────────────────────────────────────────────────────

def _camera_loop(cap):
    global latest_frame, _running
    while _running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        with nav_lock:
            dist     = nav['us_dist']
            decision = nav['decision']
            sweep    = dict(nav['sweep'])
            angle    = nav['servo_angle']
            tx       = nav['target_x']
            dtgt     = nav['dist_to_target']
            hard_stop_cm = nav['us_hard_stop_cm']
            warn_cm      = nav['us_warn_cm']

        h, w = frame.shape[:2]

        col = {'STOP':    (0,   0,   255),
               'SLOW':    (0,   165, 255),
               'ARRIVED': (0,   255, 136),
               'TURN':    (0,   170, 255),
               'IDLE':    (128, 128, 128),
               'WAIT_GPS': (180, 180, 0),
               'WAIT_HEADING': (180, 120, 0)}.get(decision, (0, 255, 0))

        # Decision banner
        label = decision if tx is None else f"{decision}  {f'{dtgt:.1f}m' if dtgt else ''}"
        cv2.putText(frame, label, (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)

        # Sweep readings top-right
        y = 30
        for a, lbl in [(45, 'L'), (90, 'C'), (135, 'R')]:
            d = sweep.get(a)
            txt = f"{lbl}({a}): {d:.0f}cm" if d else f"{lbl}({a}): N/A"
            ac = (220, 220, 0) if a == angle else (140, 140, 140)
            cv2.putText(frame, txt, (w - 195, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ac, 1)
            y += 20

        # Distance bar
        bx, by, bw = 10, h - 25, w - 20
        cv2.rectangle(frame, (bx, by), (bx + bw, by + 15), (50, 50, 50), -1)
        if dist is not None:
            dc = (0,0,255) if dist<hard_stop_cm else (0,165,255) if dist<warn_cm else (0,255,0)
            frac = min(dist / 200.0, 1.0)
            cv2.rectangle(frame, (bx, by), (bx + int(frac * bw), by + 15), dc, -1)
            cv2.putText(frame, f"front:{dist:.0f}cm", (bx+4, by-3), cv2.FONT_HERSHEY_SIMPLEX, 0.38, dc, 1)
            for thr, tl in [(hard_stop_cm,"STOP"),(warn_cm,"SLOW")]:
                tx2 = bx + int((thr/200.0)*bw)
                cv2.line(frame,(tx2,by),(tx2,by+15),(255,255,255),1)
                cv2.putText(frame,tl,(tx2-15,by-3),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)

        with frame_lock:
            latest_frame = frame


def _gps_loop(gps, heading_sensor=None):
    global _running
    prev_fix = None
    while _running:
        if gps.update():
            lat, lon = gps.get_position()
            gps_heading = None
            corrected_heading = None
            corrected_source = None

            if prev_fix is not None:
                prev_lat, prev_lon = prev_fix
                east_m, north_m = _latlon_offset_m(prev_lat, prev_lon, lat, lon)
                moved_m = math.hypot(east_m, north_m)
                if moved_m >= GPS_HEADING_MIN_MOVE_M:
                    gps_heading = _bearing_from_latlon(prev_lat, prev_lon, lat, lon)

            if gps_heading is not None and heading_sensor is not None:
                try:
                    corrected_heading = heading_sensor.correct_heading(gps_heading)
                    corrected_source = heading_sensor.nav_source
                except Exception:
                    corrected_heading = None

            with nav_lock:
                if gps_heading is not None:
                    if corrected_heading is not None:
                        nav['heading'] = corrected_heading
                        nav['heading_valid'] = True
                        nav['heading_source'] = corrected_source
                    elif not _heading_is_imu_source(nav['heading_source']):
                        nav['heading'] = gps_heading
                        nav['heading_valid'] = True
                        nav['heading_source'] = 'gps'
                nav['lat'] = lat
                nav['lon'] = lon
                _sync_target_from_gps_locked()
            prev_fix = (lat, lon)
        time.sleep(0.15)


def _imu_loop(compass):
    global _running
    while _running:
        try:
            heading = compass.read_heading()
        except Exception:
            time.sleep(0.1)
            continue

        if heading is not None:
            with nav_lock:
                nav['heading'] = heading
                nav['heading_valid'] = True
                nav['heading_source'] = compass.nav_source
        time.sleep(0.05)


# ── HTTP handler ──────────────────────────────────────────────────────────────

HTML = b"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>Rover Map</title>
<style>
:root{
  --bg:#f4f1e8;
  --panel:#fffdf8;
  --line:#d8d1c0;
  --text:#1f2a2e;
  --muted:#66747a;
  --accent:#0f766e;
  --warn:#b45309;
  --alert:#b91c1c;
}
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{
  font-family:"Avenir Next","Segoe UI",sans-serif;
  background:
    radial-gradient(circle at top left, rgba(15,118,110,0.10), transparent 34%),
    linear-gradient(180deg, #f7f3ea, #efe7d9);
  color:var(--text);
}
.page{
  min-height:100%;
  display:grid;
  grid-template-rows:auto auto 1fr auto;
  gap:12px;
  padding:12px;
}
.topbar,.stats,.panel,.footer{
  background:var(--panel);
  border:1px solid var(--line);
  border-radius:18px;
}
.topbar{
  display:flex;
  justify-content:space-between;
  align-items:center;
  padding:14px 18px;
}
.title{
  font-size:24px;
  font-weight:700;
}
.subtitle{
  color:var(--muted);
  font-size:14px;
}
.pill{
  border:1px solid #b8d0ca;
  border-radius:999px;
  padding:8px 12px;
  font:600 12px "Menlo","SFMono-Regular",monospace;
  color:var(--accent);
  background:#eef8f6;
}
.stats{
  display:grid;
  grid-template-columns:repeat(4,minmax(140px,1fr));
  gap:0;
  overflow:hidden;
}
.stat{
  padding:14px 16px;
  border-right:1px solid var(--line);
}
.stat:last-child{border-right:none}
.label{
  color:var(--muted);
  font:600 11px "Menlo","SFMono-Regular",monospace;
  text-transform:uppercase;
  letter-spacing:0.05em;
}
.value{
  margin-top:6px;
  font-size:22px;
  font-weight:700;
}
.board{
  display:grid;
  grid-template-columns:minmax(480px,1.2fr) minmax(360px,1fr);
  gap:12px;
  min-height:0;
}
.panel{
  min-height:0;
  display:flex;
  flex-direction:column;
  overflow:hidden;
}
.panel-head{
  display:flex;
  justify-content:space-between;
  align-items:center;
  padding:14px 16px;
  border-bottom:1px solid var(--line);
}
.panel-title{
  font-size:15px;
  font-weight:700;
  text-transform:uppercase;
  letter-spacing:0.04em;
}
.panel-note{
  color:var(--muted);
  font:500 12px "Menlo","SFMono-Regular",monospace;
}
.map-wrap,.camera-wrap{
  padding:14px;
  min-height:0;
  flex:1;
}
#map{
  position:relative;
  width:100%;
  height:100%;
  min-height:420px;
  border:1px solid var(--line);
  border-radius:14px;
  overflow:hidden;
  background:
    linear-gradient(0deg, rgba(255,255,255,0.15) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255,255,255,0.15) 1px, transparent 1px),
    #dce8d6;
  background-size:64px 64px, 64px 64px, auto;
  cursor:crosshair;
  user-select:none;
  touch-action:manipulation;
}
#map-tiles{
  position:absolute;
  inset:0;
}
.tile{
  position:absolute;
  width:256px;
  height:256px;
  pointer-events:none;
}
.map-status{
  position:absolute;
  left:12px;
  top:12px;
  z-index:5;
  padding:6px 10px;
  border-radius:999px;
  background:rgba(255,253,248,0.92);
  border:1px solid rgba(31,42,46,0.12);
  color:var(--muted);
  font:600 11px "Menlo","SFMono-Regular",monospace;
}
.map-crosshair{
  position:absolute;
  left:50%;
  top:50%;
  width:18px;
  height:18px;
  transform:translate(-50%,-50%);
  z-index:4;
  pointer-events:none;
}
.map-crosshair::before,
.map-crosshair::after{
  content:"";
  position:absolute;
  background:rgba(15,118,110,0.35);
}
.map-crosshair::before{
  left:8px;
  top:0;
  width:2px;
  height:18px;
}
.map-crosshair::after{
  left:0;
  top:8px;
  width:18px;
  height:2px;
}
.pin{
  position:absolute;
  z-index:6;
  pointer-events:none;
}
#rover-pin{
  width:18px;
  height:18px;
  margin-left:-9px;
  margin-top:-9px;
  border-radius:50%;
  background:#14b8a6;
  border:3px solid #0f766e;
  box-shadow:0 0 0 3px rgba(255,255,255,0.7);
}
#target-pin{
  width:20px;
  height:20px;
  margin-left:-10px;
  margin-top:-20px;
  background:#c2410c;
  border-radius:50% 50% 50% 0;
  transform:rotate(-45deg);
  box-shadow:0 2px 8px rgba(0,0,0,0.22);
}
#target-pin::after{
  content:"";
  position:absolute;
  left:5px;
  top:5px;
  width:10px;
  height:10px;
  border-radius:50%;
  background:#fff7ed;
}
.camera-stack{
  display:flex;
  flex-direction:column;
  gap:10px;
  height:100%;
}
#camimg{
  width:100%;
  height:100%;
  min-height:420px;
  object-fit:cover;
  border:1px solid var(--line);
  border-radius:14px;
  background:#111;
}
.camera-msg{
  color:var(--muted);
  font:500 12px "Menlo","SFMono-Regular",monospace;
}
.actions{
  display:flex;
  flex-wrap:wrap;
  gap:10px;
  align-items:center;
  padding:0 14px 14px;
}
.tuner{
  display:grid;
  gap:8px;
  min-width:min(100%, 320px);
  padding:12px 14px;
  border:1px solid var(--line);
  border-radius:14px;
  background:#f7f4ec;
}
.tuner-head{
  display:flex;
  justify-content:space-between;
  gap:10px;
  align-items:center;
}
.tuner-title{
  font:700 12px "Menlo","SFMono-Regular",monospace;
  color:var(--text);
  text-transform:uppercase;
  letter-spacing:0.04em;
}
.tuner-status{
  color:var(--muted);
  font:500 11px "Menlo","SFMono-Regular",monospace;
}
.slider-block{
  display:grid;
  gap:5px;
}
.slider-label{
  display:flex;
  justify-content:space-between;
  gap:10px;
  align-items:center;
  color:var(--muted);
  font:600 11px "Menlo","SFMono-Regular",monospace;
}
.slider-label strong{
  color:var(--text);
}
.slider{
  width:100%;
  accent-color:var(--accent);
}
.btn{
  border:none;
  border-radius:12px;
  padding:10px 14px;
  background:var(--accent);
  color:#fff;
  font:600 13px "Menlo","SFMono-Regular",monospace;
  cursor:pointer;
}
.btn.alt{
  background:#e9ece8;
  color:#334045;
}
.hint{
  color:var(--muted);
  font:500 12px "Menlo","SFMono-Regular",monospace;
}
.footer{
  padding:12px 16px;
  color:var(--muted);
  font:500 12px "Menlo","SFMono-Regular",monospace;
  overflow:auto;
}
@media (max-width:1100px){
  .stats{grid-template-columns:repeat(2,minmax(140px,1fr))}
  .board{grid-template-columns:1fr}
  #map,#camimg{min-height:320px}
}
</style>
</head>
<body>
<div class="page">
  <div class="topbar">
    <div>
      <div class="title">Rover Dashboard</div>
      <div class="subtitle">Tap map to set target, camera stays live</div>
    </div>
    <div class="pill" id="conn">LINK: CONNECTING</div>
  </div>

  <div class="stats">
    <div class="stat">
      <div class="label">Rover GPS</div>
      <div class="value" id="gps-val">Waiting...</div>
    </div>
    <div class="stat">
      <div class="label">Target</div>
      <div class="value" id="target-val">Not set</div>
    </div>
    <div class="stat">
      <div class="label">Decision</div>
      <div class="value" id="decision-val">IDLE</div>
    </div>
    <div class="stat">
      <div class="label">Obstacle</div>
      <div class="value" id="us-val">N/A</div>
    </div>
  </div>

  <div class="board">
    <section class="panel">
      <div class="panel-head">
        <div class="panel-title">Map</div>
        <div class="panel-note">Self-contained click map</div>
      </div>
      <div class="map-wrap">
        <div id="map">
          <div id="map-tiles"></div>
          <div id="rover-pin" class="pin" hidden></div>
          <div id="target-pin" class="pin" hidden></div>
          <div class="map-crosshair"></div>
          <div class="map-status" id="map-status">Waiting for GPS fix...</div>
        </div>
      </div>
      <div class="actions">
        <button class="btn" onclick="centerOnRover()">Center Rover</button>
        <button class="btn" onclick="zoomIn()">Zoom In</button>
        <button class="btn alt" onclick="zoomOut()">Zoom Out</button>
        <button class="btn alt" onclick="clearTarget()">Clear Target</button>
        <div class="tuner">
          <div class="tuner-head">
            <div class="tuner-title">Ultrasonic Thresholds</div>
            <div class="tuner-status" id="threshold-status">Live</div>
          </div>
          <div class="slider-block">
            <div class="slider-label">
              <span>Hard Stop</span>
              <strong id="hard-stop-value">15 cm</strong>
            </div>
            <input id="hard-stop-slider" class="slider" type="range" min="5" max="115" step="1" value="15" />
          </div>
          <div class="slider-block">
            <div class="slider-label">
              <span>Warn / Slow</span>
              <strong id="warn-value">35 cm</strong>
            </div>
            <input id="warn-slider" class="slider" type="range" min="10" max="120" step="1" value="35" />
          </div>
          <span class="hint">Slides update the rover live. Warn stays at least 5 cm above hard stop.</span>
        </div>
        <span class="hint" id="clickinfo">Tap map to set a target</span>
      </div>
    </section>

    <section class="panel">
      <div class="panel-head">
        <div class="panel-title">Camera</div>
        <div class="panel-note">Live rover view</div>
      </div>
      <div class="camera-wrap">
        <div class="camera-stack">
          <img id="camimg" src="/stream" alt="Rover camera stream" />
          <div class="camera-msg" id="cam-msg">Camera stream from /stream</div>
        </div>
      </div>
    </section>
  </div>

  <div class="footer" id="status">Waiting for rover state...</div>
</div>

<script>
const TILE_SIZE = 256;
const defaultCenter = {lat: 40.4281, lon: -86.9162};
const mapEl = document.getElementById('map');
const tilesEl = document.getElementById('map-tiles');
const roverPin = document.getElementById('rover-pin');
const targetPin = document.getElementById('target-pin');
const mapStatus = document.getElementById('map-status');
const hardStopSlider = document.getElementById('hard-stop-slider');
const warnSlider = document.getElementById('warn-slider');
const hardStopValue = document.getElementById('hard-stop-value');
const warnValue = document.getElementById('warn-value');
const thresholdStatus = document.getElementById('threshold-status');
const THRESHOLD_MIN = 5;
const THRESHOLD_MAX = 120;
const THRESHOLD_GAP = 5;

let hasCentered = false;
let tileWarned = false;
let mapView = {centerLat: defaultCenter.lat, centerLon: defaultCenter.lon, zoom: 19};
let thresholdCooldownUntil = 0;
let thresholdSendTimer = null;
let rover = {
  lat:null,
  lon:null,
  target_lat:null,
  target_lon:null,
  heading:0,
  heading_valid:false,
  heading_source:'unknown',
  decision:'IDLE',
  us_dist:null,
  us_hard_stop_cm:15,
  us_warn_cm:35,
  x:0,
  y:0,
  dist_to_target:null
};

const decisionColors = {
  IDLE:'#64748b',
  NAVIGATE:'#0f766e',
  SLOW:'#b45309',
  TURN:'#2563eb',
  STOP:'#b91c1c',
  ARRIVED:'#0f766e',
  UI_ONLY:'#a16207',
  WAIT_GPS:'#a16207',
  WAIT_HEADING:'#b45309'
};

function clamp(v, min, max){ return Math.max(min, Math.min(max, v)); }
function tileCount(z){ return 2 ** z; }
function worldSize(z){ return TILE_SIZE * tileCount(z); }

function normalizeThresholds(hardStop, warn){
  hardStop = clamp(Math.round(hardStop), THRESHOLD_MIN, THRESHOLD_MAX - THRESHOLD_GAP);
  warn = clamp(Math.round(warn), hardStop + THRESHOLD_GAP, THRESHOLD_MAX);
  hardStop = Math.min(hardStop, warn - THRESHOLD_GAP);
  return {hardStop, warn};
}

function paintThresholdValues(hardStop, warn){
  hardStopValue.textContent = `${hardStop} cm`;
  warnValue.textContent = `${warn} cm`;
}

function syncThresholdControlsFromState(){
  const {hardStop, warn} = normalizeThresholds(rover.us_hard_stop_cm, rover.us_warn_cm);
  hardStopSlider.value = hardStop;
  warnSlider.value = warn;
  paintThresholdValues(hardStop, warn);
}

async function sendThresholds(hardStop, warn){
  await fetch('/thresholds', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({hard_stop_cm: hardStop, warn_cm: warn})
  });
}

function queueThresholdUpdate(){
  const values = normalizeThresholds(Number(hardStopSlider.value), Number(warnSlider.value));
  hardStopSlider.value = values.hardStop;
  warnSlider.value = values.warn;
  paintThresholdValues(values.hardStop, values.warn);
  thresholdStatus.textContent = 'Updating...';
  thresholdCooldownUntil = Date.now() + 600;

  if (thresholdSendTimer){
    clearTimeout(thresholdSendTimer);
  }

  thresholdSendTimer = setTimeout(async () => {
    try{
      await sendThresholds(values.hardStop, values.warn);
      rover.us_hard_stop_cm = values.hardStop;
      rover.us_warn_cm = values.warn;
      thresholdStatus.textContent = 'Live';
    }catch(err){
      thresholdStatus.textContent = 'Retrying';
    }
  }, 80);
}

function latLonToWorld(lat, lon, zoom){
  const scale = worldSize(zoom);
  const sinLat = Math.sin(lat * Math.PI / 180);
  const x = ((lon + 180) / 360) * scale;
  const y = (0.5 - Math.log((1 + sinLat) / (1 - sinLat)) / (4 * Math.PI)) * scale;
  return {x, y};
}

function worldToLatLon(x, y, zoom){
  const scale = worldSize(zoom);
  const lon = (x / scale) * 360 - 180;
  const n = Math.PI - 2 * Math.PI * y / scale;
  const lat = 180 / Math.PI * Math.atan(Math.sinh(n));
  return {lat, lon};
}

function wrappedTileX(x, zoom){
  const count = tileCount(zoom);
  return ((x % count) + count) % count;
}

function getViewport(){
  const rect = mapEl.getBoundingClientRect();
  const center = latLonToWorld(mapView.centerLat, mapView.centerLon, mapView.zoom);
  return {
    rect,
    left: center.x - rect.width / 2,
    top: center.y - rect.height / 2
  };
}

function renderTiles(){
  const {rect, left, top} = getViewport();
  if (!rect.width || !rect.height){
    return;
  }

  const zoom = mapView.zoom;
  const maxTileY = tileCount(zoom) - 1;
  const x0 = Math.floor(left / TILE_SIZE);
  const x1 = Math.floor((left + rect.width) / TILE_SIZE);
  const y0 = Math.floor(top / TILE_SIZE);
  const y1 = Math.floor((top + rect.height) / TILE_SIZE);
  const frag = document.createDocumentFragment();

  for (let tx = x0; tx <= x1; tx += 1){
    for (let ty = y0; ty <= y1; ty += 1){
      if (ty < 0 || ty > maxTileY){
        continue;
      }
      const img = document.createElement('img');
      img.className = 'tile';
      img.alt = '';
      img.draggable = false;
      img.src = `https://tile.openstreetmap.org/${zoom}/${wrappedTileX(tx, zoom)}/${ty}.png`;
      img.style.left = `${Math.round(tx * TILE_SIZE - left)}px`;
      img.style.top = `${Math.round(ty * TILE_SIZE - top)}px`;
      img.addEventListener('error', () => {
        if (!tileWarned){
          tileWarned = true;
          mapStatus.textContent = 'Map tiles unavailable, but tap-to-target still works.';
        }
      });
      frag.appendChild(img);
    }
  }

  tilesEl.replaceChildren(frag);
}

function placePin(el, lat, lon, anchorBottom){
  if (lat === null || lon === null){
    el.hidden = true;
    return;
  }
  const {left, top} = getViewport();
  const point = latLonToWorld(lat, lon, mapView.zoom);
  el.hidden = false;
  el.style.left = `${point.x - left}px`;
  el.style.top = `${point.y - top}px`;
  if (anchorBottom){
    el.style.marginTop = '-20px';
  } else {
    el.style.marginTop = '-9px';
  }
}

function renderPins(){
  placePin(roverPin, rover.lat, rover.lon, false);
  placePin(targetPin, rover.target_lat, rover.target_lon, true);
}

function renderMap(){
  renderTiles();
  renderPins();
}

function fmtCoord(lat, lon){
  if(lat === null || lon === null){ return 'Waiting...'; }
  return `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
}

function fmtTarget(){
  if(rover.target_lat === null || rover.target_lon === null){ return 'Not set'; }
  return `${rover.target_lat.toFixed(6)}, ${rover.target_lon.toFixed(6)}`;
}

function fmtDistance(){
  if(rover.dist_to_target === null){ return '--'; }
  return `${rover.dist_to_target.toFixed(1)} m`;
}

function fmtObstacle(){
  if(rover.us_dist === null){ return 'N/A'; }
  return `${rover.us_dist.toFixed(0)} cm`;
}

function centerOnRover(){
  if(rover.lat !== null && rover.lon !== null){
    mapView.centerLat = rover.lat;
    mapView.centerLon = rover.lon;
    renderMap();
    mapStatus.textContent = 'Centered on rover.';
  }
}

function zoomIn(){
  mapView.zoom = clamp(mapView.zoom + 1, 16, 20);
  renderMap();
}

function zoomOut(){
  mapView.zoom = clamp(mapView.zoom - 1, 16, 20);
  renderMap();
}

async function sendTarget(lat, lon){
  await fetch('/target', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({lat, lon})
  });
}

mapEl.addEventListener('click', async (e) => {
  const rect = mapEl.getBoundingClientRect();
  const {left, top} = getViewport();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;
  const target = worldToLatLon(left + clickX, top + clickY, mapView.zoom);

  rover.target_lat = target.lat;
  rover.target_lon = target.lon;
  renderPins();
  document.getElementById('clickinfo').textContent =
    `Target set at ${target.lat.toFixed(6)}, ${target.lon.toFixed(6)}`;
  mapStatus.textContent = 'Sending target to rover...';

  try{
    await sendTarget(target.lat, target.lon);
    mapStatus.textContent = 'Target sent to rover.';
  }catch(err){
    mapStatus.textContent = 'Failed to send target.';
  }
});

hardStopSlider.addEventListener('input', () => {
  let hardStop = Number(hardStopSlider.value);
  let warn = Number(warnSlider.value);
  if (hardStop > warn - THRESHOLD_GAP){
    warn = hardStop + THRESHOLD_GAP;
  }
  const values = normalizeThresholds(hardStop, warn);
  hardStopSlider.value = values.hardStop;
  warnSlider.value = values.warn;
  paintThresholdValues(values.hardStop, values.warn);
  queueThresholdUpdate();
});

warnSlider.addEventListener('input', () => {
  let hardStop = Number(hardStopSlider.value);
  let warn = Number(warnSlider.value);
  if (warn < hardStop + THRESHOLD_GAP){
    hardStop = warn - THRESHOLD_GAP;
  }
  const values = normalizeThresholds(hardStop, warn);
  hardStopSlider.value = values.hardStop;
  warnSlider.value = values.warn;
  paintThresholdValues(values.hardStop, values.warn);
  queueThresholdUpdate();
});

async function clearTarget(){
  await fetch('/target', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({lat:null, lon:null, x:null, y:null})
  });
  rover.target_lat = null;
  rover.target_lon = null;
  renderPins();
  document.getElementById('clickinfo').textContent = 'Target cleared';
  mapStatus.textContent = 'Target cleared.';
}

async function resetOrigin(){
  await fetch('/reset', {method:'POST'});
  document.getElementById('clickinfo').textContent = 'Origin reset';
}

function paintState(){
  document.getElementById('gps-val').textContent = fmtCoord(rover.lat, rover.lon);
  document.getElementById('target-val').textContent = fmtTarget();
  const decision = document.getElementById('decision-val');
  decision.textContent = rover.decision;
  decision.style.color = decisionColors[rover.decision] || '#1f2a2e';
  document.getElementById('us-val').textContent = fmtObstacle();

  if(rover.lat !== null && rover.lon !== null && !hasCentered){
    mapView.centerLat = rover.lat;
    mapView.centerLon = rover.lon;
    hasCentered = true;
    renderMap();
    mapStatus.textContent = 'Tap map to set target.';
  } else {
    renderPins();
  }

  if (Date.now() > thresholdCooldownUntil){
    syncThresholdControlsFromState();
    thresholdStatus.textContent = 'Live';
  }

  document.getElementById('status').textContent =
    `GPS=${fmtCoord(rover.lat, rover.lon)} | ` +
    `Heading=${rover.heading.toFixed(0)} deg (${rover.heading_source}) | ` +
    `Local XY=(${rover.x.toFixed(2)}m, ${rover.y.toFixed(2)}m) | ` +
    `Distance=${fmtDistance()} | ` +
    `Obstacle=${fmtObstacle()} | ` +
    `Thresh=${rover.us_hard_stop_cm}/${rover.us_warn_cm}cm | ` +
    `Decision=${rover.decision}`;
}

async function poll(){
  try{
    const res = await fetch('/state');
    rover = await res.json();
    document.getElementById('conn').textContent = 'LINK: ONLINE';
    paintState();
  }catch(err){
    document.getElementById('conn').textContent = 'LINK: RETRYING';
    document.getElementById('status').textContent = 'Waiting for rover state...';
  }
  setTimeout(poll, 250);
}

window.addEventListener('resize', renderMap);
document.getElementById('camimg').addEventListener('error', () => {
  document.getElementById('cam-msg').textContent = 'Camera stream unavailable right now.';
});

renderMap();
syncThresholdControlsFromState();
poll();
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path

        if path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML)

        elif path == '/state':
            with nav_lock:
                data = {
                    'lat':            nav['lat'],
                    'lon':            nav['lon'],
                    'x':              nav['x'],
                    'y':              nav['y'],
                    'heading':        nav['heading'],
                    'heading_valid':  nav['heading_valid'],
                    'heading_source': nav['heading_source'],
                    'target_lat':     nav['target_lat'],
                    'target_lon':     nav['target_lon'],
                    'target_x':       nav['target_x'],
                    'target_y':       nav['target_y'],
                    'dist_to_target': nav['dist_to_target'],
                    'us_dist':        nav['us_dist'],
                    'us_hard_stop_cm': nav['us_hard_stop_cm'],
                    'us_warn_cm':      nav['us_warn_cm'],
                    'decision':       nav['decision'],
                    'trail':          nav['trail'][-200:],
                }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())

        elif path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while _running:
                with frame_lock:
                    if latest_frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpeg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                try:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
                                     jpeg.tobytes() + b'\r\n')
                except BrokenPipeError:
                    break
                time.sleep(0.05)
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length)

        if path == '/target':
            try:
                data = json.loads(body)
                with nav_lock:
                    nav['target_lat'] = data.get('lat')
                    nav['target_lon'] = data.get('lon')
                    if nav['target_lat'] is None or nav['target_lon'] is None:
                        nav['target_x'] = data.get('x')
                        nav['target_y'] = data.get('y')
                        nav['target_lat'] = None
                        nav['target_lon'] = None
                    else:
                        nav['target_x'] = None
                        nav['target_y'] = None
                        _sync_target_from_gps_locked()
                    if nav['target_x'] is None and nav['target_y'] is None:
                        nav['decision'] = 'IDLE'
                        nav['dist_to_target'] = None
                    print(
                        f"Target update: "
                        f"lat={nav['target_lat']} lon={nav['target_lon']} "
                        f"local=({nav['target_x']}, {nav['target_y']})"
                    )
            except Exception:
                pass
            self.send_response(200)
            self.end_headers()

        elif path == '/reset':
            with nav_lock:
                nav['x'] = 0.0
                nav['y'] = 0.0
                if not _heading_is_imu_source(nav['heading_source']):
                    nav['heading'] = 0.0
                    nav['heading_valid'] = False
                    nav['heading_source'] = 'unknown'
                nav['trail'] = []
                _sync_target_from_gps_locked()
            self.send_response(200)
            self.end_headers()

        elif path == '/thresholds':
            try:
                data = json.loads(body)
                with nav_lock:
                    current_hard = nav['us_hard_stop_cm']
                    current_warn = nav['us_warn_cm']
                    hard_stop = data.get('hard_stop_cm', current_hard)
                    warn = data.get('warn_cm', current_warn)
                    hard_stop, warn = _normalize_thresholds(hard_stop, warn)
                    nav['us_hard_stop_cm'] = hard_stop
                    nav['us_warn_cm'] = warn
                    print(f"Ultrasonic thresholds updated: stop={hard_stop}cm warn={warn}cm")
            except Exception:
                pass
            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


# ── Drive loop ────────────────────────────────────────────────────────────────

def _drive_loop(motors, sonar, heading_sensor=None):
    global _running, _sweep_pause

    interval  = 1.0 / LOOP_HZ
    last_time = time.monotonic()

    while _running:
        now = time.monotonic()
        dt  = now - last_time
        last_time = now

        with nav_lock:
            x, y      = nav['x'], nav['y']
            heading   = nav['heading']
            heading_valid = nav['heading_valid']
            heading_source = nav['heading_source']
            imu_enabled = nav['imu_enabled']
            lat, lon  = nav['lat'], nav['lon']
            target_lat = nav['target_lat']
            target_lon = nav['target_lon']
            tx        = nav['target_x']
            ty        = nav['target_y']
            front_dist = nav['us_dist']
            hard_stop_cm = nav['us_hard_stop_cm']
            warn_cm = nav['us_warn_cm']

        # ── Obstacle check ────────────────────────────────────────────────────
        obstacle_stop = front_dist is not None and front_dist < hard_stop_cm
        obstacle_slow = front_dist is not None and front_dist < warn_cm

        # ── No target → idle ──────────────────────────────────────────────────
        if target_lat is None and tx is None:
            motors.stop()
            with nav_lock:
                nav['decision']       = 'IDLE'
                nav['dist_to_target'] = None
            time.sleep(interval)
            continue

        # ── Prefer true GPS navigation when we have both a fix and a GPS target ──
        if None not in (lat, lon, target_lat, target_lon):
            dist_to_target = get_distance_meters(lat, lon, target_lat, target_lon)
            bearing = get_bearing(lat, lon, target_lat, target_lon)
            with nav_lock:
                _sync_target_from_gps_locked()
                tx = nav['target_x']
                ty = nav['target_y']
        else:
            if tx is None or ty is None:
                motors.stop()
                with nav_lock:
                    nav['decision'] = 'WAIT_GPS'
                    nav['dist_to_target'] = None
                time.sleep(interval)
                continue
            dist_to_target = math.hypot(tx - x, ty - y)
            bearing = _bearing(x, y, tx, ty)

        h_error = _heading_error(heading, bearing)

        with nav_lock:
            nav['dist_to_target'] = dist_to_target
            if not heading_valid:
                if imu_enabled and heading_sensor is not None:
                    try:
                        seeded_heading = heading_sensor.seed_heading(bearing, absolute=False)
                    except Exception:
                        seeded_heading = None
                    if seeded_heading is not None:
                        nav['heading'] = seeded_heading
                        nav['heading_valid'] = True
                        nav['heading_source'] = heading_sensor.nav_source
                        heading = seeded_heading
                        h_error = _heading_error(heading, bearing)
                    else:
                        nav['decision'] = 'WAIT_HEADING'
                        nav['heading_source'] = 'unknown'
                        motors.stop()
                        time.sleep(interval)
                        continue
                elif imu_enabled:
                    nav['decision'] = 'WAIT_HEADING'
                    nav['heading_source'] = 'unknown'
                    motors.stop()
                    time.sleep(interval)
                    continue
                else:
                    nav['heading'] = bearing
                    nav['heading_valid'] = True
                    nav['heading_source'] = 'estimated'
                    heading = bearing
                    h_error = 0.0

        preferred_turn = None
        if h_error < -HEADING_TOL:
            preferred_turn = 'left'
        elif h_error > HEADING_TOL:
            preferred_turn = 'right'

        # ── Arrived ───────────────────────────────────────────────────────────
        if dist_to_target < ARRIVE_M:
            motors.stop()
            with nav_lock:
                nav['decision'] = 'ARRIVED'
            if None not in (target_lat, target_lon):
                print(f"Arrived near GPS target ({target_lat:.6f}, {target_lon:.6f})")
            else:
                print(f"Arrived at target ({tx:.2f}m, {ty:.2f}m)")
            time.sleep(interval)
            continue

        # ── Obstacle handling: fresh scan, then decide whether to turn ────────
        if obstacle_stop or obstacle_slow:
            with nav_lock:
                nav['decision'] = 'STOP' if obstacle_stop else 'SLOW'

            readings = _adaptive_scan(motors, sonar)
            center_dist = readings.get(CRUISE_SERVO_ANGLE)
            center_clear = _clearance_value(center_dist)
            turn = _choose_turn_direction(readings, preferred_turn)
            side_clear = _clearance_value(readings.get(45 if turn == 'left' else 135))
            should_turn = (
                obstacle_stop or
                center_clear < hard_stop_cm or
                side_clear > center_clear + TURN_PREFERENCE_CM or
                abs(h_error) > HEADING_TOL
            )

            if should_turn:
                display_dist = front_dist if front_dist is not None else center_dist
                display_txt = f"{display_dist:.0f}cm" if display_dist is not None else "N/A"
                if obstacle_stop or center_clear < hard_stop_cm:
                    print(f"Obstacle {display_txt} ahead — backing up then turning {turn}")
                    motors.drive_backward()
                    time.sleep(AVOID_BACKUP_S)
                else:
                    print(f"Obstacle ahead in warning zone — turning {turn}")

                if turn == 'left':
                    motors.turn_left()
                    with nav_lock:
                        if not _heading_is_imu_source(nav['heading_source']):
                            nav['heading'] = (nav['heading'] - TURN_DPS * AVOID_TURN_S) % 360
                            nav['heading_valid'] = True
                            nav['heading_source'] = 'estimated'
                        nav['decision'] = 'TURN'
                else:
                    motors.turn_right()
                    with nav_lock:
                        if not _heading_is_imu_source(nav['heading_source']):
                            nav['heading'] = (nav['heading'] + TURN_DPS * AVOID_TURN_S) % 360
                            nav['heading_valid'] = True
                            nav['heading_source'] = 'estimated'
                        nav['decision'] = 'TURN'

                time.sleep(AVOID_TURN_S)
                motors.stop()
                last_time = time.monotonic()
                continue

            motors.drive_forward(DRIVE_SPEED_SLOW)
            spd = SLOW_MPS
            with nav_lock:
                nav['decision'] = 'SLOW'

        # ── Slow zone: drive forward slowly ───────────────────────────────────
        if obstacle_stop or obstacle_slow:
            pass
        # ── Need to turn toward target ────────────────────────────────────────
        elif abs(h_error) > HEADING_TOL:
            if h_error < 0:
                motors.turn_left()
                with nav_lock:
                    if not _heading_is_imu_source(nav['heading_source']):
                        nav['heading'] = (nav['heading'] - TURN_DPS * dt) % 360
                        nav['heading_valid'] = True
                        nav['heading_source'] = 'estimated'
                    nav['decision'] = 'TURN'
            else:
                motors.turn_right()
                with nav_lock:
                    if not _heading_is_imu_source(nav['heading_source']):
                        nav['heading'] = (nav['heading'] + TURN_DPS * dt) % 360
                        nav['heading_valid'] = True
                        nav['heading_source'] = 'estimated'
                    nav['decision'] = 'TURN'
            time.sleep(interval)
            last_time = time.monotonic()
            continue
        # ── Clear path, drive toward target ───────────────────────────────────
        else:
            speed = DRIVE_SPEED_SLOW if dist_to_target < 0.8 else DRIVE_SPEED_FULL
            motors.drive_forward(speed)
            spd = SLOW_MPS if dist_to_target < 0.8 else FORWARD_MPS
            with nav_lock:
                nav['decision'] = 'SLOW' if dist_to_target < 0.8 else 'NAVIGATE'

        # ── Update dead reckoning position ────────────────────────────────────
        with nav_lock:
            h_rad    = math.radians(nav['heading'])
            nav['x'] += math.sin(h_rad) * spd * dt
            nav['y'] += math.cos(h_rad) * spd * dt
            _add_trail(nav['x'], nav['y'])

        time.sleep(interval)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _running

    print("=== Rover Map Navigation ===")
    print(f"Open http://10.10.10.90:{PORT} in your browser")
    print("Click the map to send the rover to a location.")
    print("Ctrl+C to stop.\n")

    drive_enabled = True
    try:
        motors = MotorController()
        motors.send_command('V90')
        print(f"Motors: OK ({motors.port})")
    except Exception as exc:
        motors = _NullMotorController()
        drive_enabled = False
        with nav_lock:
            nav['decision'] = 'UI_ONLY'
        print(f"Motors: unavailable ({exc})")
        print("Starting UI-only mode. Map and camera stay available.")

    try:
        sonar = UltrasonicSensor()
        print("Ultrasonic: OK")
    except Exception as exc:
        sonar = _NullUltrasonicSensor()
        print(f"Ultrasonic: unavailable ({exc})")

    try:
        gps = GPSReader()
        print("GPS: OK")
    except Exception as exc:
        gps = None
        print(f"GPS: unavailable ({exc})")

    try:
        compass = MPU9250Heading()
        with nav_lock:
            nav['imu_enabled'] = True
        print(
            f"Heading IMU: OK ({compass.core_name}, core WHO_AM_I=0x{compass.core_id:02X}, "
            f"/dev/i2c-{compass.bus_id} at 0x{compass.address:02X}, mode={compass.mode}, "
            f"gyro_axis={compass.gyro_axis_name}, "
            f"{compass.mag_status}, gyro_bias={compass.gyro_bias_dps:.2f} dps)"
        )
    except Exception as exc:
        compass = None
        with nav_lock:
            nav['imu_enabled'] = False
        print(f"Heading IMU: unavailable ({exc})")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if cap.isOpened():
        for _ in range(5):
            cap.read()
        print("Camera: OK")
    else:
        print("Camera: not found")

    if drive_enabled:
        Thread(target=_sweep_loop,  args=(motors, sonar), daemon=True).start()
    if gps is not None:
        Thread(target=_gps_loop, args=(gps, compass), daemon=True).start()
    if compass is not None:
        Thread(target=_imu_loop, args=(compass,), daemon=True).start()
    Thread(target=_camera_loop, args=(cap,),           daemon=True).start()

    server = ThreadingHTTPServer(('0.0.0.0', PORT), _Handler)
    Thread(target=server.serve_forever, daemon=True).start()

    print("Ready.\n")

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        if drive_enabled:
            _drive_loop(motors, sonar, compass)
        else:
            while _running:
                time.sleep(0.25)
    finally:
        _running = False
        motors.send_command('V90')
        motors.stop()
        motors.close()
        sonar.close()
        if gps is not None:
            gps.close()
        if compass is not None:
            compass.close()
        cap.release()
        server.shutdown()
        print("\nStopped.")


if __name__ == '__main__':
    main()
