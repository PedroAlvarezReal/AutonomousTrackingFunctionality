#!/usr/bin/env python3
# ── rover/test_motors.py ─────────────────────────────────────────────────────
# Drives both motors forward for 2 seconds then stops.
# Run this FIRST to confirm wiring before connecting any sensors.
#
# Usage:   sudo python3 test_motors.py
# Sudo required: sysfs GPIO export needs root on most Linux configs.
#
# If a motor spins the wrong direction, swap the two wires at that motor's
# L298N output terminals (OUT1/OUT2 or OUT3/OUT4) — do NOT change the code.

import time
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController


def main() -> None:
    print("=== Motor test ===")
    print("Driving forward at full speed for 2 seconds...")

    motors = MotorController()
    try:
        motors.drive_forward(100)
        time.sleep(2)
    finally:
        motors.stop()
        motors.close()
        print("Stopped. GPIO released.")


if __name__ == "__main__":
    main()
