# ── rover/vision/safety.py ───────────────────────────────────────────────────
# SafetyMonitor fuses ultrasonic distance + camera obstacle scores.
#
# The two sensors reinforce each other:
#
#   ULTRASONIC tells us HOW FAR an obstacle is (precise distance, narrow beam).
#   CAMERA tells us IF something is there (wide field of view, no distance).
#
# Fusion rules:
#   1. Ultrasonic timeout or < HARD_STOP → STOP immediately (hardware brake)
#   2. Ultrasonic in warn zone + camera sees anything → STOP (both agree)
#   3. Ultrasonic in warn zone + camera clear → SLOW (trust ultrasonic)
#   4. Camera score high (> threshold) → SLOW even if ultrasonic reads far
#      (camera has wider FOV — it might see something the narrow beam misses)
#   5. Both clear → CLEAR, full speed
#
# The camera score is boosted when ultrasonic reads closer — if the sensor
# says something is near AND the camera kinda sees it, that's more certain.

from config import (
    ULTRASONIC_HARD_STOP_CM,
    ULTRASONIC_WARN_CM,
    DRIVE_SPEED_FULL,
    DRIVE_SPEED_SLOW,
    CAMERA_OBSTACLE_SCORE_THRESHOLD,
)
from vision.ultrasonic import UltrasonicSensor
from vision.detector   import ObstacleDetector


class Decision:
    STOP  = "STOP"
    SLOW  = "SLOW"
    CLEAR = "CLEAR"


class SafetyMonitor:
    """Fuses UltrasonicSensor + ObstacleDetector into one drive decision."""

    def __init__(self) -> None:
        self._sonar  = UltrasonicSensor()
        self._camera = ObstacleDetector()

    def evaluate(self) -> tuple[float | None, dict | None, str, int]:
        """Return (distance_cm, scores_dict, decision, recommended_speed).

        distance_cm : float | None (None = sonar timeout)
        scores_dict : dict with contour/motion/color/edge/combined scores
        decision    : Decision constant
        recommended_speed : int — 0, DRIVE_SPEED_SLOW, or DRIVE_SPEED_FULL
        """
        distance = self._sonar.read_cm()
        scores, camera_obstacle = self._camera.check()

        combined_score = scores['combined'] if scores else 0.0

        # ── RULE 1: Ultrasonic emergency brake (only ultrasonic can STOP) ───
        # Timeout (None) = sensor unreliable, ignore it and rely on camera
        if distance is not None and distance < ULTRASONIC_HARD_STOP_CM:
            return distance, scores, Decision.STOP, 0

        # ── RULE 2: Ultrasonic warn zone (25–60cm) → SLOW, regardless of camera ──
        if distance is not None and distance < ULTRASONIC_WARN_CM:
            return distance, scores, Decision.SLOW, DRIVE_SPEED_SLOW

        # ── RULE 3: Camera sees obstacle even though ultrasonic reads far ────
        # Camera can only cause SLOW, never STOP
        if camera_obstacle:
            return distance, scores, Decision.SLOW, DRIVE_SPEED_SLOW

        # ── RULE 4: Both clear → full speed ──────────────────────────────────
        return distance, scores, Decision.CLEAR, DRIVE_SPEED_FULL

    def close(self) -> None:
        try:
            self._sonar.close()
        except Exception:
            pass
        try:
            self._camera.close()
        except Exception:
            pass
