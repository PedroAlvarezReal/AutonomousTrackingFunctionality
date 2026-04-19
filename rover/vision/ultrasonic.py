# ── rover/vision/ultrasonic.py ───────────────────────────────────────────────
# HC-SR04 distance sensor via sysfs GPIO.
#
# Timing diagram:
#   1. Pull TRIG HIGH for ≥ 10 µs → sensor fires 8 × 40 kHz bursts
#   2. ECHO goes HIGH for exactly the round-trip travel time
#   3. distance = echo_duration × (speed_of_sound / 2)
#
# ECHO line safety:
#   The HC-SR04 ECHO pin outputs 5 V.  A 1 kΩ + 2 kΩ voltage divider
#   on the rover board drops it to 3.3 V before it reaches the Pi GPIO.
#   Do NOT remove that divider — the Rubik Pi GPIO is 3.3 V tolerant only.

import os
import time

from config import PIN_TRIG, PIN_ECHO, ULTRASONIC_TIMEOUT_S

_GPIO_ROOT = "/sys/class/gpio"


def _write(path: str, value) -> None:
    with open(path, "w") as f:
        f.write(str(value))


def _read(path: str) -> str:
    with open(path, "r") as f:
        return f.read().strip()


def _export(pin: int, direction: str) -> None:
    pin_path = f"{_GPIO_ROOT}/gpio{pin}"
    if not os.path.exists(pin_path):
        _write(f"{_GPIO_ROOT}/export", pin)
        time.sleep(0.01)  # sysfs node needs a moment to appear
    _write(f"{pin_path}/direction", direction)


def _unexport(pin: int) -> None:
    if os.path.exists(f"{_GPIO_ROOT}/gpio{pin}"):
        _write(f"{_GPIO_ROOT}/unexport", pin)


class UltrasonicSensor:
    """Reads distance from an HC-SR04 sensor over sysfs GPIO.

    Returns None on timeout — callers MUST treat None as an obstacle
    (fail-safe, not fail-open).
    """

    def __init__(self) -> None:
        self._trig = f"{_GPIO_ROOT}/gpio{PIN_TRIG}/value"
        self._echo = f"{_GPIO_ROOT}/gpio{PIN_ECHO}/value"
        self._setup_gpio()

    def _setup_gpio(self) -> None:
        _export(PIN_TRIG, "out")
        _export(PIN_ECHO, "in")

        # settle the trigger line; the sensor ignores spurious pulses
        # during the first 50 ms after power-on
        _write(self._trig, 0)
        time.sleep(0.05)

    def _ensure_ready(self) -> None:
        if os.path.exists(self._trig) and os.path.exists(self._echo):
            return
        self._setup_gpio()

    def _read_cm_once(self) -> float | None:
        self._ensure_ready()

        # send the 10 µs trigger pulse
        _write(self._trig, 1)
        time.sleep(0.00001)
        _write(self._trig, 0)

        deadline = time.monotonic() + ULTRASONIC_TIMEOUT_S

        # wait for ECHO to go HIGH (pulse start)
        while _read(self._echo) == "0":
            if time.monotonic() > deadline:
                return None  # sensor didn't respond → treat as obstacle

        pulse_start = time.monotonic()

        # wait for ECHO to go LOW (pulse end)
        while _read(self._echo) == "1":
            if time.monotonic() > deadline:
                return None  # echo stuck HIGH → treat as obstacle

        pulse_end = time.monotonic()

        # speed of sound ≈ 34 300 cm/s; divide by 2 for round-trip
        return round((pulse_end - pulse_start) * 17150, 1)

    def read_cm(self) -> float | None:
        """Return distance in centimetres, or None if echo timed out."""
        try:
            return self._read_cm_once()
        except (FileNotFoundError, OSError):
            # Some runs lose the sysfs node mid-flight. Re-export once and
            # retry so navigation does not crash on a transient GPIO reset.
            try:
                self._setup_gpio()
                return self._read_cm_once()
            except Exception:
                return None

    def close(self) -> None:
        try:
            if os.path.exists(self._trig):
                _write(self._trig, 0)
        except Exception:
            pass
        try:
            _unexport(PIN_TRIG)
        except Exception:
            pass
        try:
            _unexport(PIN_ECHO)
        except Exception:
            pass
