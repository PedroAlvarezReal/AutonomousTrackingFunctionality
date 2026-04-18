#!/usr/bin/env python3
# ── rover/main.py ────────────────────────────────────────────────────────────
# Phase 1: forward driving with collision avoidance.
# GPS waypoint navigation is out of scope for this phase — see gps/ for that.
#
# Usage:   sudo python3 main.py
# Ctrl+C or SIGTERM will stop the motors and release all resources cleanly.
#
# Loop rate is set by LOOP_HZ in config.py (default 10 Hz).
# Each iteration prints one status line:
#   🔊 42.3 cm | 👁 0.120 | CLEAR  → drive full speed (100%)
#   🔊 38.0 cm | 👁 0.088 | SLOW   → drive slow (50%) | ultrasonic warn
#   🔊 18.5 cm | 👁 N/A   | STOP   → stopped | ultrasonic critical
#   🔊 TIMEOUT | 👁 N/A   | STOP   → stopped | sonar timeout

import time
import signal
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController
from vision.safety     import SafetyMonitor, Decision
from config            import LOOP_HZ, ULTRASONIC_HARD_STOP_CM, ULTRASONIC_WARN_CM, CAMERA_EDGE_THRESHOLD


def _reason(dist, density, decision) -> str:
    """Build a short human-readable cause string for the status line."""
    if dist is None:
        return "sonar timeout"
    if decision == Decision.STOP:
        if dist < ULTRASONIC_HARD_STOP_CM:
            return "ultrasonic critical"
        if density is not None and density > CAMERA_EDGE_THRESHOLD:
            return "camera obstacle"
    if decision == Decision.SLOW:
        return "ultrasonic warn"
    return ""


def main() -> None:
    print("=== Autonomous rover — phase 1: collision avoidance ===")
    print("Ctrl+C to stop safely.\n")

    # Both objects must be created before motors are allowed to spin.
    # If either raises (bad GPIO number, no camera), the except block
    # ensures we never enter the drive loop.
    motors  = None
    monitor = None

    try:
        motors  = MotorController()
        monitor = SafetyMonitor()
    except Exception as exc:
        print(f"FATAL: sensor/motor init failed — {exc}")
        print("Motors will NOT run.  Fix the error and restart.")
        if motors:
            motors.close()
        if monitor:
            monitor.close()
        sys.exit(1)

    running  = True
    interval = 1.0 / LOOP_HZ

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while running:
            dist, density, decision, speed = monitor.evaluate()

            dist_str    = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"
            density_str = f"{density:.3f}" if density is not None else "N/A"

            if decision == Decision.CLEAR:
                motors.drive_forward(speed)
                label = f"drive full speed ({speed}%)"
            elif decision == Decision.SLOW:
                motors.drive_forward(speed)
                label = f"drive slow ({speed}%)"
            else:
                motors.stop()
                label = "stopped"

            reason = _reason(dist, density, decision)
            suffix = f" | {reason}" if reason else ""

            print(
                f"🔊 {dist_str:<10} | 👁 {density_str} | "
                f"{decision:<5} → {label}{suffix}"
            )

            time.sleep(interval)

    finally:
        motors.stop()
        motors.close()
        monitor.close()
        print("\nRover stopped. All resources released.")


if __name__ == "__main__":
    main()
