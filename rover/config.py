# ── rover/config.py ──────────────────────────────────────────────────────────
# Single source of truth for every threshold, pin, and tunable constant.
# No magic numbers should appear anywhere else in the codebase.

# ─── Phase 1: Collision avoidance ────────────────────────────────────────────

# Ultrasonic thresholds
ULTRASONIC_HARD_STOP_CM = 25    # below this → hard stop, no questions asked
ULTRASONIC_WARN_CM      = 60    # 25–60 cm → slow down, still moving
ULTRASONIC_TIMEOUT_S    = 0.04  # no echo received within this window → treat as obstacle (fail safe)

# Camera / edge detection
CAMERA_EDGE_THRESHOLD    = 0.15  # fraction of pixels flagged as edges → obstacle present
CAMERA_REGION_HEIGHT_PCT = 0.40  # bottom 40 % of frame = the zone we're about to enter
CAMERA_REGION_WIDTH_PCT  = 0.50  # centre 50 % horizontally (ignores peripheral clutter)
CAMERA_INDEX             = 0     # /dev/video0

# Motor speeds  (0–100 %)
DRIVE_SPEED_FULL = 100
DRIVE_SPEED_SLOW = 50

# Main loop rate
LOOP_HZ = 10

# Software PWM carrier frequency for ENA/ENB lines
# sysfs GPIO can't do hardware PWM, so we toggle in a thread at this rate.
PWM_FREQUENCY_HZ = 500   # 500 Hz is plenty for L298N speed control

# ─── GPIO pin mapping ─────────────────────────────────────────────────────────
# Every PIN_* constant is the sysfs GPIO number that gets written to
# /sys/class/gpio/export.  These are NOT the physical pin numbers.
#
# HOW TO FIND YOUR SYSFS NUMBERS on the Rubik Pi 3:
#   sudo cat /sys/kernel/debug/gpio
# Look for lines like:
#   gpio-NNN (GPIO_8): ...
# The NNN is what goes below.
#
# Alternatively:  gpioinfo   (from libgpiod-tools)
#
# Column guide:
#   Constant   GPIO name  Physical pin   sysfs number
# ─────────────────────────────────────────────────────
# TODO: replace every -1 with the real number from your board.

# L298N motor driver
PIN_IN1 = -1   # GPIO_8  → pin 11
PIN_IN2 = -1   # GPIO_24 → pin 13
PIN_IN3 = -1   # GPIO_25 → pin 15
PIN_IN4 = -1   # GPIO_26 → pin 16
PIN_ENA = -1   # GPIO_27 → pin 18  (Motor A enable, software PWM)
PIN_ENB = -1   # GPIO_12 → pin 27  (Motor B enable, software PWM)

# HC-SR04 ultrasonic sensor
PIN_TRIG = 32  # GPIO_32 → pin 29
PIN_ECHO = 33  # GPIO_33 → pin 31  (ECHO stepped down to 3.3 V via 1 kΩ + 2 kΩ divider)

# ─── Phase 2: GPS navigation (out of scope for phase 1) ──────────────────────
GPS_PORT           = '/dev/ttyS0'
GPS_BAUD           = 9600
ARRIVAL_THRESHOLD  = 2.5   # metres — how close counts as "arrived"
HEADING_TOLERANCE  = 10    # degrees off-bearing before we correct
DRIVE_SPEED        = 60    # legacy constant used by GPS nav code
TURN_SPEED         = 40
SIMULATION_MODE    = True
