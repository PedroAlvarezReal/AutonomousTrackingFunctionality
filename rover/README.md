# Rover — Autonomous Collision Avoidance (Phase 1)

Phase 1: the rover drives forward continuously and stops/slows when sensors
detect an obstacle.  GPS waypoint navigation is Phase 2 (see `gps/`).

---

## Install dependencies

```bash
pip3 install opencv-python numpy
# GPS phase (optional for phase 1):
pip3 install pyserial pynmea2
```

---

## Find your sysfs GPIO numbers  ← do this before anything else

The `config.py` file has `PIN_* = -1` placeholders.  You must replace them
with real sysfs numbers from your Rubik Pi 3.

```bash
sudo cat /sys/kernel/debug/gpio
```

Look for lines like:
```
 gpio-488 (GPIO_8 ): output active-high
 gpio-504 (GPIO_24): output active-high
```
The number after `gpio-` is your sysfs number.  Fill in `config.py`:
```python
PIN_IN1 = 488   # GPIO_8  → pin 11
PIN_IN2 = 504   # GPIO_24 → pin 13
# … etc
```

Alternatively, if `libgpiod-tools` is installed:
```bash
gpioinfo
```

---

## Testing order  (follow this exactly)

### Step 1 — Motors alone
```bash
sudo python3 test_motors.py
```
Both wheels should spin forward for 2 seconds then stop.
If one spins backward, swap its wires at the L298N output terminals.

### Step 2 — Ultrasonic sensor alone
```bash
sudo python3 test_ultrasonic.py
```
Hold your hand at various distances and watch the readings.
- Clear (> 60 cm) → 🟢
- 25–60 cm → 🟡 WARN
- < 25 cm → 🔴 HARD STOP
- Cover sensor → TIMEOUT (correct: treated as obstacle)

### Step 3 — Camera / OpenCV alone
```bash
python3 test_camera.py
```
No sudo needed.  Saves `test_frame_01.jpg … test_frame_05.jpg` in the
current directory.  Open them to check:
- Green rectangle shows the danger zone (bottom-centre crop)
- Edge density printed for each frame
- Adjust `CAMERA_EDGE_THRESHOLD` in `config.py` if needed

### Step 4 — Safety layer (no motors)
```bash
sudo python3 test_safety.py
```
Tune thresholds here — the motors never move during this test.
Watch the printed decisions while waving obstacles at the sensor/camera.

### Step 5 — Full drive loop
```bash
sudo python3 main.py
```
Rover drives forward.  Place an obstacle in front to verify it stops.
Ctrl+C stops safely.

---

## Commands that need sudo and why

| Command | Reason |
|---|---|
| `test_motors.py` | Writing to `/sys/class/gpio/export` requires root |
| `test_ultrasonic.py` | Same — sysfs GPIO export |
| `test_safety.py` | Same |
| `main.py` | Same |
| `test_camera.py` | Does NOT need sudo — OpenCV uses `/dev/video0` directly |

---

## Troubleshooting

### "Permission denied" on `/sys/class/gpio/export`
```bash
sudo python3 your_script.py
```
Or add your user to the `gpio` group (if it exists on your distro):
```bash
sudo usermod -aG gpio $USER
```

### "No such device" / GPIO export fails
- Verify your sysfs numbers with `sudo cat /sys/kernel/debug/gpio`
- The `-1` placeholder in `config.py` will immediately raise an error —
  this is intentional so the rover never starts with bad pin config

### "Already exported" error on re-run
The code handles this gracefully — it checks if the gpio path exists
before writing to `export`.  If you still see it, a previous run crashed
mid-export.  Clean up with:
```bash
echo <sysfs_number> | sudo tee /sys/class/gpio/unexport
```

### Motors spin the wrong direction
Swap the two wires at the L298N output terminals for that motor
(OUT1/OUT2 for Motor A, OUT3/OUT4 for Motor B).
Do NOT change `IN1/IN2` order in the wiring or in code.

### Camera not found (`RuntimeError: Cannot open camera`)
```bash
ls /dev/video*          # confirm the device exists
v4l2-ctl --list-devices # see what's attached
```
Change `CAMERA_INDEX` in `config.py` if the camera is on `/dev/video1`, etc.

### Edge density always 0
- Check that the lens cap is off
- Make sure the camera is actually capturing (test_camera.py saves frames)

### Edge density always above threshold
- The scene may just be very textured (carpet, grass)
- Raise `CAMERA_EDGE_THRESHOLD` in `config.py` (try 0.20 or 0.25)

---

## Tuning constants (`config.py`)

| Constant | Default | Effect |
|---|---|---|
| `ULTRASONIC_HARD_STOP_CM` | `25` | Distance that triggers immediate stop |
| `ULTRASONIC_WARN_CM` | `60` | Distance that triggers slow speed |
| `ULTRASONIC_TIMEOUT_S` | `0.04` | Echo wait — increase if you get spurious timeouts |
| `CAMERA_EDGE_THRESHOLD` | `0.15` | Lower = more sensitive; raise for textured floors |
| `CAMERA_REGION_HEIGHT_PCT` | `0.40` | How much of the bottom of the frame is the danger zone |
| `CAMERA_REGION_WIDTH_PCT` | `0.50` | How wide the danger zone is |
| `DRIVE_SPEED_FULL` | `100` | Full forward speed (%) |
| `DRIVE_SPEED_SLOW` | `50` | Warn-zone speed (%) |
| `LOOP_HZ` | `10` | Main loop rate |

---

## Phase 2 — GPS navigation

The `gps/` directory contains the navigation code from the earlier phase.
Set `SIMULATION_MODE = True` in `config.py` to run without hardware.

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from gps.simulator import GPSSimulator
from gps.navigator import get_distance_meters
s = GPSSimulator(41.8827, -87.6233)
print('GPS sim works')
"
```
