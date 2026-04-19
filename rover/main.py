#!/usr/bin/env python3
# ── rover/main.py ────────────────────────────────────────────────────────────
# Phase 1: forward driving with collision avoidance + servo scan-on-demand.
#
# Strategy:
#   1. Drive forward with ultrasonic pointing straight ahead (servo 90°).
#   2. When an obstacle is detected (ultrasonic or camera), STOP.
#   3. Do a 3-point servo sweep (left / center / right) to find the
#      direction with the most clearance.
#   4. Turn toward the best opening, resume driving.
#
# Usage:   sudo python3 main.py        (needs GPIO for ultrasonic)
#          python3 main.py              (falls back to motors-only)
# Ctrl+C or SIGTERM will stop the motors and release all resources cleanly.

import time
import signal
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from motors.controller import MotorController
from vision.safety     import SafetyMonitor, Decision
from vision.servo_sweep import ServoSweep
from vision.ultrasonic import UltrasonicSensor
from config            import (
    LOOP_HZ, ULTRASONIC_HARD_STOP_CM, ULTRASONIC_WARN_CM,
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
)

# How long to turn when avoiding an obstacle (seconds)
TURN_DURATION = 0.8


def _reason(dist, scores, decision) -> str:
    """Build a short human-readable cause string for the status line."""
    if dist is None:
        return "sonar timeout"
    if decision == Decision.STOP:
        if dist < ULTRASONIC_HARD_STOP_CM:
            return "ultrasonic critical"
        if scores and scores['combined'] > CAMERA_OBSTACLE_SCORE_THRESHOLD:
            return "camera + ultrasonic"
    if decision == Decision.SLOW:
        if dist < ULTRASONIC_WARN_CM:
            return "ultrasonic warn"
        if scores and scores['combined'] > CAMERA_OBSTACLE_SCORE_THRESHOLD:
            return "camera obstacle"
    return ""


def _scan_and_turn(motors: MotorController, sweep: ServoSweep) -> None:
    """Stop, do a 3-point scan, turn toward the clearest direction."""
    motors.stop()
    print("🔍 Scanning for best direction...")

    readings = sweep.quick_scan()  # {0: dist, 90: dist, 180: dist}
    print(f"   Left(0°)={readings.get(0)} cm  "
          f"Center(90°)={readings.get(90)} cm  "
          f"Right(180°)={readings.get(180)} cm")

    best_angle, best_dist = sweep.best_direction(readings)

    if best_angle < 30:
        # Best clearance is straight ahead — back up and try left
        print("⬅️  Forward blocked too — backing up and turning left")
        motors.drive_backward()
        time.sleep(0.5)
        motors.turn_left()
        time.sleep(TURN_DURATION)
    elif best_angle <= 90:
        # Best clearance is to the right
        print(f"↪️  Turning RIGHT (best at {best_angle}°, {best_dist} cm)")
        motors.turn_right()
        time.sleep(TURN_DURATION)
    else:
        # Best clearance is far right — bigger turn
        print(f"↪️  Turning HARD RIGHT (best at {best_angle}°, {best_dist} cm)")
        motors.turn_right()
        time.sleep(TURN_DURATION * 1.5)

    motors.stop()


def main() -> None:
    print("=== Autonomous rover — phase 1: collision avoidance ===")
    print("Strategy: drive forward, scan-on-demand when obstacle detected")
    print("Ctrl+C to stop safely.\n")

    motors  = None
    monitor = None
    sweep   = None

    try:
        motors = MotorController()
        # Center servo on startup
        motors.send_command("V90")

        try:
            monitor = SafetyMonitor()
            # Create sweep using the motor serial + the ultrasonic sensor
            us = UltrasonicSensor()
            sweep = ServoSweep(motors, us)
        except Exception as exc:
            print(f"⚠️  Sensors unavailable ({exc}) — running MOTORS ONLY mode")
            monitor = None
            sweep = None
    except Exception as exc:
        print(f"FATAL: motor init failed — {exc}")
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
            if monitor:
                dist, scores, decision, speed = monitor.evaluate()

                dist_str = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"
                if scores:
                    score_str = (f"C:{scores['contour']:.2f} M:{scores['motion']:.2f} "
                                 f"L:{scores['color']:.2f} E:{scores['edge']:.2f} "
                                 f"= {scores['combined']:.2f}")
                else:
                    score_str = "N/A"

                reason = _reason(dist, scores, decision)
                suffix = f" | {reason}" if reason else ""

                if decision == Decision.CLEAR:
                    motors.drive_forward(speed)
                    label = f"drive full ({speed}%)"
                elif decision == Decision.SLOW:
                    motors.drive_forward(speed)
                    label = f"drive slow ({speed}%)"
                else:
                    # ── OBSTACLE → scan and pick best direction ──────────
                    label = "OBSTACLE — scanning"
                    print(
                        f"🔊 {dist_str:<10} | 👁 {score_str} | "
                        f"{decision:<5} → {label}{suffix}"
                    )
                    if sweep:
                        _scan_and_turn(motors, sweep)
                    else:
                        # No servo — just stop and wait
                        motors.stop()
                        time.sleep(1.0)
                    continue  # re-evaluate immediately after manoeuvre

                print(
                    f"🔊 {dist_str:<10} | 👁 {score_str} | "
                    f"{decision:<5} → {label}{suffix}"
                )
            else:
                # Motors-only mode: just drive forward
                motors.drive_forward(100)
                print("🚗 MOTORS ONLY — driving forward (no sensors)")

            time.sleep(interval)

    finally:
        motors.send_command("V0")  # re-center servo (straight ahead)
        motors.stop()
        motors.close()
        if monitor:
            monitor.close()
        print("\nRover stopped. All resources released.")


if __name__ == "__main__":
    main()
