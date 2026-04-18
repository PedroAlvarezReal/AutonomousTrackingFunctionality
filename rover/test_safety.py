#!/usr/bin/env python3
# ── rover/test_safety.py ─────────────────────────────────────────────────────
# Runs the full safety decision loop and prints every decision — NO motors.
# Use this to tune thresholds on your desk before ever running main.py.
#
# Usage:   sudo python3 test_safety.py
# Ctrl+C to exit cleanly.
#
# Tuning guide:
#   1. Clear path — should print CLEAR with speed=100%
#   2. Hand at ~40 cm — should print SLOW with speed=50%
#   3. Hand at ~15 cm — should print STOP (ultrasonic tier)
#   4. Dense object just out of sonar range — should print STOP (camera tier)
#   5. If camera fires too eagerly → raise CAMERA_EDGE_THRESHOLD in config.py
#   6. If camera misses real obstacles → lower CAMERA_EDGE_THRESHOLD

import time
import signal
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from vision.safety import SafetyMonitor, Decision
from config import LOOP_HZ


def main() -> None:
    print("=== Safety monitor test  (NO motor movement) ===")
    print("Ctrl+C to quit.\n")

    monitor = SafetyMonitor()
    running = True

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)

    interval = 1.0 / LOOP_HZ

    try:
        while running:
            dist, density, decision, speed = monitor.evaluate()

            dist_str    = f"{dist:.1f} cm" if dist is not None else "TIMEOUT"
            density_str = f"{density:.3f}" if density is not None else "N/A"

            if decision == Decision.STOP:
                icon = "🔴"
            elif decision == Decision.SLOW:
                icon = "🟡"
            else:
                icon = "🟢"

            print(
                f"{icon}  🔊 {dist_str:>10}  |  👁 {density_str}  |  "
                f"{decision:<5}  →  speed {speed}%"
            )

            time.sleep(interval)
    finally:
        monitor.close()
        print("\nSafety monitor released.")


if __name__ == "__main__":
    main()
