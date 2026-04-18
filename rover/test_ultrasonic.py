#!/usr/bin/env python3
# ── rover/test_ultrasonic.py ─────────────────────────────────────────────────
# Reads the HC-SR04 continuously and prints distance every 0.3 s.
# Run this SECOND, after motors are confirmed working.
#
# Usage:   sudo python3 test_ultrasonic.py
# Ctrl+C to exit cleanly.
#
# What to look for:
#   - Clear path (> 60 cm)  → numbers in the 60–400 cm range
#   - Hold your hand 30 cm away → should read ~30 cm
#   - Block the sensor completely → should print TIMEOUT (correct behaviour)
#   - Erratic large jumps → check your voltage divider on the ECHO line

import time
import signal
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from vision.ultrasonic import UltrasonicSensor
from config import ULTRASONIC_HARD_STOP_CM, ULTRASONIC_WARN_CM


def main() -> None:
    print("=== Ultrasonic sensor test ===")
    print(f"Thresholds:  HARD STOP < {ULTRASONIC_HARD_STOP_CM} cm  |  WARN < {ULTRASONIC_WARN_CM} cm")
    print("Ctrl+C to quit.\n")

    sensor  = UltrasonicSensor()
    running = True

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    try:
        while running:
            dist = sensor.read_cm()

            if dist is None:
                status = "⚠  TIMEOUT — no echo (treated as obstacle)"
            elif dist < ULTRASONIC_HARD_STOP_CM:
                status = f"🔴 {dist:.1f} cm — HARD STOP zone"
            elif dist < ULTRASONIC_WARN_CM:
                status = f"🟡 {dist:.1f} cm — WARN / slow zone"
            else:
                status = f"🟢 {dist:.1f} cm — clear"

            print(status)
            time.sleep(0.3)
    finally:
        sensor.close()
        print("\nSensor released.")


if __name__ == "__main__":
    main()
