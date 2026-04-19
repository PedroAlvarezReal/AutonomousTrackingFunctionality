# ── rover/config.py ──────────────────────────────────────────────────────────
# Single source of truth for every threshold, pin, and tunable constant.
# No magic numbers should appear anywhere else in the codebase.

# ─── Phase 1: Collision avoidance ────────────────────────────────────────────

# Ultrasonic thresholds
ULTRASONIC_HARD_STOP_CM = 25    # below this → hard stop, no questions asked
ULTRASONIC_WARN_CM      = 60    # 25–60 cm → slow down, still moving
ULTRASONIC_TIMEOUT_S    = 0.04  # no echo received within this window → treat as obstacle (fail safe)

# Camera / multi-method obstacle detection
# Combined score from contour + motion + colour + edge signals
CAMERA_OBSTACLE_SCORE_THRESHOLD = 0.55  # 0.0–1.0 — combined score above this = obstacle
CAMERA_REGION_HEIGHT_PCT = 0.40  # bottom 40 % of frame = the zone we're about to enter
CAMERA_REGION_WIDTH_PCT  = 0.50  # centre 50 % horizontally (ignores peripheral clutter)
CAMERA_INDEX             = 0     # /dev/video0
CAMERA_CONTOUR_MIN_AREA_PCT = 0.05  # ignore contours smaller than 5% of danger zone (noise)
CAMERA_MOTION_THRESHOLD     = 40    # pixel intensity diff to count as motion (0–255)
CAMERA_COLOR_DIFF_THRESHOLD = 90    # weighted HSV diff to count as "not floor"

# Motor speeds  (0–100 %)
DRIVE_SPEED_FULL = 100
DRIVE_SPEED_SLOW = 50

# Main loop rate
LOOP_HZ = 10

# ─── Arduino serial (motor control) ──────────────────────────────────────────
# The Rubik Pi sends text commands over USB to an Arduino Uno, which drives
# the L298N H-bridge.  The Pi does NOT touch motor pins directly.
#
# Wiring:  Pi USB-A port  ──USB cable──►  Arduino USB-B port
#   (this also powers the Arduino from the Pi)
#
# Find the port on the Pi with:   ls /dev/ttyACM* /dev/ttyUSB*
ARDUINO_SERIAL_PORT = "/dev/ttyUSB0"     # Arduino Uno via USB
ARDUINO_BAUD_RATE   = 9600

# ─── GPIO pin mapping (Rubik Pi 3 sysfs) ─────────────────────────────────────
# sysfs number = gpiochip base (547) + GPIO offset

# HC-SR04 ultrasonic sensor
PIN_TRIG = 579  # GPIO_32 → pin 29  (547 + 32)
PIN_ECHO = 580  # GPIO_33 → pin 31  (547 + 33, ECHO stepped down to 3.3 V via 1 kΩ + 2 kΩ divider)

# ─── Phase 2: GPS navigation (out of scope for phase 1) ──────────────────────
GPS_PORT           = '/dev/ttyS0'
GPS_BAUD           = 9600
ARRIVAL_THRESHOLD  = 2.5   # metres — how close counts as "arrived"
HEADING_TOLERANCE  = 10    # degrees off-bearing before we correct
DRIVE_SPEED        = 60    # legacy constant used by GPS nav code
TURN_SPEED         = 40
SIMULATION_MODE    = True
