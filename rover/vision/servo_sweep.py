"""Servo sweep – scan ultrasonic at multiple angles via Arduino servo."""

import time
import logging
from rover.motors.controller import MotorController
from rover.vision.ultrasonic import UltrasonicSensor

log = logging.getLogger(__name__)

# Angles to scan (degrees): center(forward) → right → far-right
# 0° = straight ahead, 90° = right, 180° = behind-right
SWEEP_ANGLES = [0, 45, 90, 135, 180]
SETTLE_TIME  = 0.3  # seconds for servo to reach position


class ServoSweep:
    """Send V<angle> commands to Arduino and read ultrasonic at each angle."""

    def __init__(self, motor_ctrl: MotorController, ultrasonic: UltrasonicSensor):
        self.motor = motor_ctrl
        self.us = ultrasonic
        self._current_angle = 0

    # ── low-level ────────────────────────────────────────────────────────

    def set_angle(self, angle: int):
        """Move servo to *angle* (0-180) via Arduino serial."""
        angle = max(0, min(180, angle))
        self.motor.send_command(f"V{angle}")
        self._current_angle = angle
        time.sleep(SETTLE_TIME)

    def center(self):
        self.set_angle(0)

    # ── sweep ────────────────────────────────────────────────────────────

    def sweep(self, angles=None):
        """Scan ultrasonic at each angle.

        Returns dict {angle: distance_cm} (None for timeouts).
        """
        if angles is None:
            angles = SWEEP_ANGLES

        readings: dict[int, float | None] = {}
        for ang in angles:
            self.set_angle(ang)
            dist = self.us.read_distance()
            readings[ang] = dist
            log.debug("sweep %3d° → %s cm", ang, dist)

        # Return to center after sweep
        self.center()
        return readings

    def quick_scan(self):
        """3-point scan: left-center-right.  Fastest useful scan."""
        return self.sweep([0, 90, 180])  # forward, right, far-right

    def best_direction(self, readings: dict):
        """Pick the angle with the most clearance.

        Returns (best_angle, distance) or (90, None) if all timed out.
        """
        valid = {a: d for a, d in readings.items() if d is not None}
        if not valid:
            return 0, None
        best = max(valid, key=valid.get)
        return best, valid[best]
