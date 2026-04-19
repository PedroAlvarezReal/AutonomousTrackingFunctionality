#!/usr/bin/env python3
# ── rover/test_motors.py ─────────────────────────────────────────────────────
# Sends serial commands to Arduino Uno to test all motor directions.
# The Arduino drives the L298N H-bridge.
#
# Usage:   python3 test_motors.py

import time
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController


def main() -> None:
    print("=== Motor test (Arduino USB serial) ===")
    print("⚠️  MOTORS WILL SPIN — lift wheels off the ground!\n")
    time.sleep(2)

    motors = MotorController()
    try:
        print("▶ FORWARD (2s)")
        motors.drive_forward()
        time.sleep(2)

        print("▶ STOP (1s)")
        motors.stop()
        time.sleep(1)

        print("▶ BACKWARD (2s)")
        motors.drive_backward()
        time.sleep(2)

        print("▶ STOP (1s)")
        motors.stop()
        time.sleep(1)

        print("▶ LEFT (1.5s)")
        motors.turn_left()
        time.sleep(1.5)

        print("▶ STOP (0.5s)")
        motors.stop()
        time.sleep(0.5)

        print("▶ RIGHT (1.5s)")
        motors.turn_right()
        time.sleep(1.5)

        print("▶ STOP")
        motors.stop()

    finally:
        motors.close()

    print("\n✅ Motor test complete!")


if __name__ == "__main__":
    main()
