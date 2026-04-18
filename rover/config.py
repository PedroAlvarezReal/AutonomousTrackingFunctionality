# ── All your settings live here ──────────────────────────────

# GPS hardware port — change if yours is different
GPS_PORT = '/dev/ttyS0'
GPS_BAUD = 9600

# How close to the target before we call it "arrived" (meters)
ARRIVAL_THRESHOLD = 2.5

# How many degrees off-bearing before we correct the turn
HEADING_TOLERANCE = 10

# Motor speeds (0-100)
DRIVE_SPEED = 60
TURN_SPEED  = 40

# Simulation mode — set True when testing without hardware
SIMULATION_MODE = True
