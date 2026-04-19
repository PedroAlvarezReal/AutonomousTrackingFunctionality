#!/usr/bin/env python3
# ── rover/map_drive.py ───────────────────────────────────────────────────────
# Interactive map navigation + live camera stream.
#
# Click anywhere on the map to send the rover to that location.
# Rover navigates using dead reckoning (estimates position from motor
# commands + time). Obstacle avoidance runs continuously via servo sweep.
# Camera streams live at all times.
#
# Stream: http://10.10.10.90:8080
# Usage:  sudo /home/ubuntu/AutonomousTrackingFunctionality/venv/bin/python3 map_drive.py

import cv2, sys, os, time, signal, math, json
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread, Lock
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController
from vision.ultrasonic  import UltrasonicSensor
from config import (
    LOOP_HZ, CAMERA_INDEX,
    ULTRASONIC_HARD_STOP_CM, ULTRASONIC_WARN_CM,
    DRIVE_SPEED_FULL, DRIVE_SPEED_SLOW,
)

PORT = 8080

# ── Tunable constants ─────────────────────────────────────────────────────────
FORWARD_MPS  = 0.25    # estimated m/s at full speed — calibrate after testing
SLOW_MPS     = 0.12    # estimated m/s at slow speed
TURN_DPS     = 110.0   # estimated degrees/sec when turning in place
ARRIVE_M     = 0.35    # metres — close enough to count as arrived
HEADING_TOL  = 12      # degrees — acceptable heading error before driving
TRAIL_STEP   = 0.15    # metres — minimum distance before adding trail point
SWEEP_ANGLES = [45, 90, 135]
SWEEP_SETTLE = 0.15    # seconds for servo to reach position

# ── Shared state ──────────────────────────────────────────────────────────────
nav = {
    'x': 0.0, 'y': 0.0,           # position in metres from start
    'heading': 0.0,                # degrees: 0=North, 90=East, 180=South, 270=West
    'target_x': None,
    'target_y': None,
    'dist_to_target': None,
    'us_dist': None,               # closest ultrasonic reading
    'servo_angle': 90,
    'sweep': {45: None, 90: None, 135: None},
    'decision': 'IDLE',
    'trail': [],
}
nav_lock     = Lock()
latest_frame = None
frame_lock   = Lock()
_running     = True
_sweep_pause = False   # paused during backup/turns


# ── Helpers ───────────────────────────────────────────────────────────────────

def _bearing(x1, y1, x2, y2):
    """Compass bearing from (x1,y1) to (x2,y2). 0=North, clockwise."""
    dx, dy = x2 - x1, y2 - y1
    return math.degrees(math.atan2(dx, dy)) % 360


def _heading_error(current, target):
    """Signed heading error in degrees. Negative = turn left, positive = turn right."""
    return (target - current + 180) % 360 - 180


def _add_trail(x, y):
    trail = nav['trail']
    if trail:
        lx, ly = trail[-1]
        if math.hypot(x - lx, y - ly) < TRAIL_STEP:
            return
    trail.append((x, y))
    if len(trail) > 500:
        trail.pop(0)


# ── Servo sweep thread ────────────────────────────────────────────────────────

def _sweep_loop(motors, sonar):
    global _running, _sweep_pause
    idx = 0
    while _running:
        if _sweep_pause:
            time.sleep(0.05)
            continue
        angle = SWEEP_ANGLES[idx % len(SWEEP_ANGLES)]
        motors.send_command(f"V{angle}")
        time.sleep(SWEEP_SETTLE)
        if _sweep_pause:
            idx += 1
            continue
        dist = sonar.read_cm()
        with nav_lock:
            nav['sweep'][angle]  = dist
            nav['servo_angle']   = angle
            valid = [d for d in nav['sweep'].values() if d is not None]
            nav['us_dist'] = min(valid) if valid else None
        idx += 1


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

        h, w = frame.shape[:2]

        col = {'STOP':    (0,   0,   255),
               'SLOW':    (0,   165, 255),
               'ARRIVED': (0,   255, 136),
               'TURN':    (0,   170, 255),
               'IDLE':    (128, 128, 128)}.get(decision, (0, 255, 0))

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
            dc = (0,0,255) if dist<ULTRASONIC_HARD_STOP_CM else (0,165,255) if dist<ULTRASONIC_WARN_CM else (0,255,0)
            frac = min(dist / 200.0, 1.0)
            cv2.rectangle(frame, (bx, by), (bx + int(frac * bw), by + 15), dc, -1)
            cv2.putText(frame, f"min:{dist:.0f}cm", (bx+4, by-3), cv2.FONT_HERSHEY_SIMPLEX, 0.38, dc, 1)
            for thr, tl in [(ULTRASONIC_HARD_STOP_CM,"STOP"),(ULTRASONIC_WARN_CM,"SLOW")]:
                tx2 = bx + int((thr/200.0)*bw)
                cv2.line(frame,(tx2,by),(tx2,by+15),(255,255,255),1)
                cv2.putText(frame,tl,(tx2-15,by-3),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)

        with frame_lock:
            latest_frame = frame


# ── HTTP handler ──────────────────────────────────────────────────────────────

HTML = b"""<!DOCTYPE html>
<html>
<head>
<title>Rover Map</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#eee;font-family:monospace;height:100vh;display:flex;flex-direction:column;overflow:hidden}
#top{display:flex;flex:1;gap:8px;padding:8px;min-height:0}
#map-panel{display:flex;flex-direction:column;gap:4px;flex:0 0 auto}
#map-label{color:#00ff88;font-size:13px}
#map{border:2px solid #00ff88;cursor:crosshair;display:block}
#camera-panel{flex:1;display:flex;flex-direction:column;gap:4px;min-width:0}
#cam-label{color:#00ff88;font-size:13px}
#camimg{width:100%;flex:1;object-fit:contain;border:2px solid #00ff88;min-height:0}
#status{background:#1a1a1a;padding:6px 10px;font-size:12px;border-top:1px solid #333;white-space:nowrap;overflow:hidden}
#hint{font-size:11px;color:#666}
.btn{background:#222;border:1px solid #555;color:#eee;padding:4px 10px;cursor:pointer;font-family:monospace;font-size:12px}
.btn:hover{background:#333}
#controls{display:flex;gap:6px;align-items:center;margin-top:4px}
</style>
</head>
<body>
<div id="top">
  <div id="map-panel">
    <div id="map-label">MAP &mdash; click to set target &nbsp;<span id="hint">(1 square = 1 metre)</span></div>
    <canvas id="map" width="460" height="460"></canvas>
    <div id="controls">
      <button class="btn" onclick="clearTarget()">Clear target</button>
      <button class="btn" onclick="resetOrigin()">Reset origin</button>
      <span id="clickinfo" style="color:#888;font-size:11px"></span>
    </div>
  </div>
  <div id="camera-panel">
    <div id="cam-label">LIVE CAMERA</div>
    <img id="camimg" src="/stream" alt="stream"/>
  </div>
</div>
<div id="status">Loading...</div>

<script>
const canvas = document.getElementById('map');
const ctx    = canvas.getContext('2d');
const W = canvas.width, H = canvas.height;
const SCALE = 46;  // px per metre
const CX = W/2, CY = H/2;

let rover = {x:0,y:0,heading:0,target_x:null,target_y:null,
             dist_to_target:null,us_dist:null,decision:'IDLE',trail:[]};

function w2c(wx,wy){return [CX+wx*SCALE, CY-wy*SCALE];}
function c2w(cx,cy){return [(cx-CX)/SCALE, -(cy-CY)/SCALE];}

function drawMap(){
  ctx.clearRect(0,0,W,H);

  // Grid
  ctx.strokeStyle='#262626'; ctx.lineWidth=1;
  for(let x=CX%SCALE;x<W;x+=SCALE){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,H);ctx.stroke();}
  for(let y=CY%SCALE;y<H;y+=SCALE){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}

  // Axis
  ctx.strokeStyle='#333'; ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(CX,0);ctx.lineTo(CX,H);ctx.stroke();
  ctx.beginPath();ctx.moveTo(0,CY);ctx.lineTo(W,CY);ctx.stroke();

  // Metre labels
  ctx.fillStyle='#444'; ctx.font='10px monospace';
  for(let m=-4;m<=4;m++){if(!m)continue;
    let [cx]=w2c(m,0); ctx.fillText(m+'m',cx-8,CY+14);
    let [,cy]=w2c(0,m); ctx.fillText(m+'m',CX+4,cy+4);
  }

  // Trail
  if(rover.trail.length>1){
    ctx.strokeStyle='rgba(0,220,100,0.35)'; ctx.lineWidth=2;
    ctx.beginPath();
    let[sx,sy]=w2c(rover.trail[0][0],rover.trail[0][1]); ctx.moveTo(sx,sy);
    rover.trail.forEach(([tx,ty])=>{let[cx,cy]=w2c(tx,ty);ctx.lineTo(cx,cy);});
    ctx.stroke();
  }

  // Target
  if(rover.target_x!==null){
    let[tx,ty]=w2c(rover.target_x,rover.target_y);
    let arrived=rover.decision==='ARRIVED';
    ctx.fillStyle=arrived?'#00ff88':'rgba(255,60,60,0.25)';
    ctx.beginPath();ctx.arc(tx,ty,10,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=arrived?'#00ff88':'#ff4444'; ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(tx,ty,10,0,Math.PI*2);ctx.stroke();
    // crosshair
    ctx.beginPath();ctx.moveTo(tx-7,ty);ctx.lineTo(tx+7,ty);
    ctx.moveTo(tx,ty-7);ctx.lineTo(tx,ty+7);ctx.stroke();
  }

  // Rover arrow
  let[rx,ry]=w2c(rover.x,rover.y);
  let decCols={CLEAR:'#00ee00',SLOW:'#ffa500',STOP:'#ff3333',
               TURN:'#00aaff',NAVIGATE:'#00ee00',ARRIVED:'#00ff88',IDLE:'#777'};
  ctx.save();
  ctx.translate(rx,ry);
  ctx.rotate((rover.heading-90)*Math.PI/180); // canvas 0=East, we want 0=North
  ctx.fillStyle=decCols[rover.decision]||'#888';
  ctx.beginPath();
  ctx.moveTo(0,-14); ctx.lineTo(9,9); ctx.lineTo(0,4); ctx.lineTo(-9,9);
  ctx.closePath(); ctx.fill();
  ctx.strokeStyle='rgba(255,255,255,0.5)'; ctx.lineWidth=1; ctx.stroke();
  ctx.restore();
}

canvas.addEventListener('click',e=>{
  const r=canvas.getBoundingClientRect();
  const[wx,wy]=c2w(e.clientX-r.left, e.clientY-r.top);
  fetch('/target',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({x:wx,y:wy})});
  document.getElementById('clickinfo').textContent=`Target: (${wx.toFixed(1)}m, ${wy.toFixed(1)}m)`;
});

function clearTarget(){
  fetch('/target',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({x:null,y:null})});
  document.getElementById('clickinfo').textContent='Target cleared';
}

function resetOrigin(){
  fetch('/reset',{method:'POST'});
  document.getElementById('clickinfo').textContent='Origin reset';
}

async function poll(){
  try{
    const r=await fetch('/state');
    rover=await r.json();
    drawMap();
    const us=rover.us_dist!==null?rover.us_dist.toFixed(0)+'cm':'N/A';
    const dt=rover.dist_to_target!==null?rover.dist_to_target.toFixed(2)+'m':'--';
    const decCols={CLEAR:'#00ee00',SLOW:'#ffa500',STOP:'#ff4444',
                   TURN:'#00aaff',NAVIGATE:'#00cc00',ARRIVED:'#00ff88',IDLE:'#888'};
    const dc=decCols[rover.decision]||'#eee';
    document.getElementById('status').innerHTML=
      `Pos: <b>(${rover.x.toFixed(2)}m, ${rover.y.toFixed(2)}m)</b> &nbsp;|&nbsp; `+
      `Heading: <b>${rover.heading.toFixed(0)}&deg;</b> &nbsp;|&nbsp; `+
      `US: <b>${us}</b> &nbsp;|&nbsp; `+
      `To target: <b>${dt}</b> &nbsp;|&nbsp; `+
      `Decision: <b style="color:${dc}">${rover.decision}</b>`;
  }catch(e){}
  setTimeout(poll,150);
}
drawMap();poll();
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
                    'x':              nav['x'],
                    'y':              nav['y'],
                    'heading':        nav['heading'],
                    'target_x':       nav['target_x'],
                    'target_y':       nav['target_y'],
                    'dist_to_target': nav['dist_to_target'],
                    'us_dist':        nav['us_dist'],
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
                    nav['target_x'] = data.get('x')
                    nav['target_y'] = data.get('y')
                    nav['decision'] = 'IDLE' if data.get('x') is None else nav['decision']
            except Exception:
                pass
            self.send_response(200)
            self.end_headers()

        elif path == '/reset':
            with nav_lock:
                nav['x'] = 0.0
                nav['y'] = 0.0
                nav['heading'] = 0.0
                nav['trail'] = []
            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


# ── Drive loop ────────────────────────────────────────────────────────────────

def _drive_loop(motors):
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
            tx        = nav['target_x']
            ty        = nav['target_y']
            us_dist   = nav['us_dist']

        # ── Obstacle check ────────────────────────────────────────────────────
        obstacle_stop = us_dist is not None and us_dist < ULTRASONIC_HARD_STOP_CM
        obstacle_slow = us_dist is not None and us_dist < ULTRASONIC_WARN_CM

        # ── No target → idle ──────────────────────────────────────────────────
        if tx is None:
            motors.stop()
            with nav_lock:
                nav['decision']       = 'IDLE'
                nav['dist_to_target'] = None
            time.sleep(interval)
            continue

        # ── Distance + bearing to target ──────────────────────────────────────
        dist_to_target = math.hypot(tx - x, ty - y)
        bearing        = _bearing(x, y, tx, ty)
        h_error        = _heading_error(heading, bearing)

        with nav_lock:
            nav['dist_to_target'] = dist_to_target

        # ── Arrived ───────────────────────────────────────────────────────────
        if dist_to_target < ARRIVE_M:
            motors.stop()
            with nav_lock:
                nav['decision'] = 'ARRIVED'
            print(f"Arrived at target ({tx:.2f}m, {ty:.2f}m)")
            time.sleep(interval)
            continue

        # ── Obstacle: back up and turn toward clearest side ───────────────────
        if obstacle_stop:
            _sweep_pause = True
            with nav_lock:
                sweep = dict(nav['sweep'])
                nav['decision'] = 'STOP'
            left  = sweep.get(45)  or 0
            right = sweep.get(135) or 0
            turn  = 'left' if left >= right else 'right'
            print(f"Obstacle {us_dist:.0f}cm — back+{turn}")

            motors.drive_backward()
            time.sleep(0.4)
            if turn == 'left':
                motors.turn_left()
                with nav_lock:
                    nav['heading'] = (nav['heading'] - TURN_DPS * 0.6) % 360
            else:
                motors.turn_right()
                with nav_lock:
                    nav['heading'] = (nav['heading'] + TURN_DPS * 0.6) % 360
            time.sleep(0.6)
            motors.stop()

            motors.send_command('V90')
            time.sleep(0.15)
            with nav_lock:
                nav['sweep'] = {45: None, 90: None, 135: None}
                nav['us_dist'] = None
            _sweep_pause = False

            last_time = time.monotonic()
            continue

        # ── Slow zone: drive forward slowly ───────────────────────────────────
        if obstacle_slow:
            motors.drive_forward(DRIVE_SPEED_SLOW)
            spd = SLOW_MPS
            with nav_lock:
                nav['decision'] = 'SLOW'
        # ── Need to turn toward target ────────────────────────────────────────
        elif abs(h_error) > HEADING_TOL:
            if h_error < 0:
                motors.turn_left()
                with nav_lock:
                    nav['heading'] = (nav['heading'] - TURN_DPS * dt) % 360
                    nav['decision'] = 'TURN'
            else:
                motors.turn_right()
                with nav_lock:
                    nav['heading'] = (nav['heading'] + TURN_DPS * dt) % 360
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

    motors = MotorController()
    motors.send_command('V90')

    sonar = UltrasonicSensor()
    print("Ultrasonic: OK")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if cap.isOpened():
        for _ in range(5):
            cap.read()
        print("Camera: OK")
    else:
        print("Camera: not found")

    Thread(target=_sweep_loop,  args=(motors, sonar), daemon=True).start()
    Thread(target=_camera_loop, args=(cap,),           daemon=True).start()

    server = HTTPServer(('0.0.0.0', PORT), _Handler)
    Thread(target=server.serve_forever, daemon=True).start()

    print("Ready.\n")

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        _drive_loop(motors)
    finally:
        _running = False
        motors.send_command('V90')
        motors.stop()
        motors.close()
        sonar.close()
        cap.release()
        server.shutdown()
        print("\nStopped.")


if __name__ == '__main__':
    main()
