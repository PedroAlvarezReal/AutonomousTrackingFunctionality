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

        # ── RULE 1: Ultrasonic emergency brake ───────────────────────────────
        if distance is None or distance < ULTRASONIC_HARD_STOP_CM:
            return distance, scores, Decision.STOP, 0

        # ── Boost camera score when ultrasonic says something is near ────────
        # If ultrasonic reads 30–60 cm and camera score is borderline,
        # the proximity makes us more confident it's real
        if distance < ULTRASONIC_WARN_CM:
            # Scale boost: closer = bigger boost (up to 1.5x at 25 cm)
            proximity_factor = 1.0 + 0.5 * (1.0 - (distance - ULTRASONIC_HARD_STOP_CM)
                                              / (ULTRASONIC_WARN_CM - ULTRASONIC_HARD_STOP_CM))
            boosted_score = min(1.0, combined_score * proximity_factor)
        else:
            boosted_score = combined_score

        # ── RULE 2: Ultrasonic warn zone + camera confirms → STOP ────────────
        if distance < ULTRASONIC_WARN_CM and boosted_score > CAMERA_OBSTACLE_SCORE_THRESHOLD:
            return distance, scores, Decision.STOP, 0

        # ── RULE 3: Ultrasonic warn zone + camera clear → SLOW ───────────────
        if distance < ULTRASONIC_WARN_CM:
            return distance, scores, Decision.SLOW, DRIVE_SPEED_SLOW

        # ── RULE 4: Camera sees obstacle even though ultrasonic reads far ────
        # Camera has wider FOV — it might catch something the narrow beam misses
        if camera_obstacle:
            return distance, scores, Decision.SLOW, DRIVE_SPEED_SLOW

        # ── RULE 5: Both clear → full speed ──────────────────────────────────
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
