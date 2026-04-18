# ── rover/vision/safety.py ───────────────────────────────────────────────────
# SafetyMonitor fuses the ultrasonic and camera layers into one decision.
#
# Priority hierarchy (strict — higher tier always wins):
#
#   TIER 1 — Ultrasonic (hardware emergency brake)
#     distance == None (timeout) → STOP   ← fail safe, not fail open
#     distance  < HARD_STOP_CM  → STOP
#     distance  < WARN_CM       → in warn zone (SLOW unless camera also flags)
#     distance  ≥ WARN_CM       → defer entirely to TIER 2
#
#   TIER 2 — Camera (wide-area awareness)
#     obstacle_detected          → STOP
#     clear                      → CLEAR (or SLOW if still in warn zone)
#
# The camera is ONLY consulted when TIER 1 has not already forced a stop.
# This keeps the ultrasonic as a dumb-but-reliable last resort.

from config import (
    ULTRASONIC_HARD_STOP_CM,
    ULTRASONIC_WARN_CM,
    DRIVE_SPEED_FULL,
    DRIVE_SPEED_SLOW,
)
from vision.ultrasonic import UltrasonicSensor
from vision.detector   import ObstacleDetector


class Decision:
    STOP  = "STOP"
    SLOW  = "SLOW"
    CLEAR = "CLEAR"


class SafetyMonitor:
    """Owns one UltrasonicSensor and one ObstacleDetector.

    Call evaluate() every loop tick.  It is safe to call close() in a
    finally block even if __init__ raised partway through — each sub-object
    is guarded individually.
    """

    def __init__(self) -> None:
        self._sonar  = UltrasonicSensor()
        self._camera = ObstacleDetector()

    def evaluate(self) -> tuple[float | None, float | None, str, int]:
        """Return (distance_cm, edge_density, decision, recommended_speed).

        distance_cm       : float | None  (None = sonar timeout)
        edge_density      : float | None  (None = camera not consulted or failed)
        decision          : Decision constant
        recommended_speed : int — 0, DRIVE_SPEED_SLOW, or DRIVE_SPEED_FULL
        """
        distance = self._sonar.read_cm()

        # ── TIER 1 ───────────────────────────────────────────────────────────
        if distance is None or distance < ULTRASONIC_HARD_STOP_CM:
            return distance, None, Decision.STOP, 0

        in_warn_zone = distance < ULTRASONIC_WARN_CM

        # ── TIER 2 ───────────────────────────────────────────────────────────
        edge_density, obstacle_seen = self._camera.check()

        if obstacle_seen:
            return distance, edge_density, Decision.STOP, 0

        if in_warn_zone:
            return distance, edge_density, Decision.SLOW, DRIVE_SPEED_SLOW

        return distance, edge_density, Decision.CLEAR, DRIVE_SPEED_FULL

    def close(self) -> None:
        try:
            self._sonar.close()
        except Exception:
            pass
        try:
            self._camera.close()
        except Exception:
            pass
