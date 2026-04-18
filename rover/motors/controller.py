# ── rover/motors/controller.py ───────────────────────────────────────────────
# L298N dual H-bridge via sysfs GPIO.
# Only forward motion is implemented in phase 1 — turning comes with GPS nav.
#
# Software PWM note:
#   The Rubik Pi sysfs interface does not expose hardware PWM on these pins,
#   so speed control is done by toggling the ENA/ENB lines in a background
#   thread at PWM_FREQUENCY_HZ.  At 100 % duty the pin is held HIGH and the
#   thread does no toggling, so full-speed has zero CPU overhead.
#   TODO (phase 2): wire ENB/ENA to a real hardware PWM channel if one is
#   available, and replace _SoftPWM with a thin sysfs PWM wrapper.

import os
import time
import threading

from config import (
    PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4,
    PIN_ENA, PIN_ENB,
    PWM_FREQUENCY_HZ,
)

_GPIO_ROOT = "/sys/class/gpio"


# ── low-level sysfs helpers ───────────────────────────────────────────────────

def _write(path: str, value) -> None:
    with open(path, "w") as f:
        f.write(str(value))


def _export(pin: int, direction: str = "out") -> None:
    pin_path = f"{_GPIO_ROOT}/gpio{pin}"
    if not os.path.exists(pin_path):
        _write(f"{_GPIO_ROOT}/export", pin)
        # brief pause — the sysfs node isn't always ready immediately
        time.sleep(0.01)
    _write(f"{pin_path}/direction", direction)
    _write(f"{pin_path}/value", 0)


def _unexport(pin: int) -> None:
    if os.path.exists(f"{_GPIO_ROOT}/gpio{pin}"):
        _write(f"{_GPIO_ROOT}/unexport", pin)


def _set(pin: int, value: int) -> None:
    _write(f"{_GPIO_ROOT}/gpio{pin}/value", value)


# ── software PWM ─────────────────────────────────────────────────────────────

class _SoftPWM:
    """Toggles a sysfs GPIO pin at a fixed frequency with a variable duty cycle."""

    def __init__(self, pin: int, frequency_hz: int) -> None:
        self._pin    = pin
        self._period = 1.0 / frequency_hz
        self._duty   = 0.0   # 0.0–1.0
        self._lock   = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread.start()

    def set_duty(self, pct: float) -> None:
        """Accept 0–100 and store as 0.0–1.0."""
        with self._lock:
            self._duty = max(0.0, min(100.0, pct)) / 100.0

    def _run(self) -> None:
        while self._running:
            with self._lock:
                duty = self._duty

            if duty <= 0.0:
                _set(self._pin, 0)
                time.sleep(self._period)
            elif duty >= 1.0:
                # no toggling needed at full speed — just keep HIGH
                _set(self._pin, 1)
                time.sleep(self._period)
            else:
                on_time  = self._period * duty
                off_time = self._period - on_time
                _set(self._pin, 1)
                time.sleep(on_time)
                _set(self._pin, 0)
                time.sleep(off_time)

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        _set(self._pin, 0)


# ── public interface ──────────────────────────────────────────────────────────

class MotorController:
    """
    Controls two DC motors wired to an L298N H-bridge.

    Motor A = left wheel  (IN1 / IN2 / ENA)
    Motor B = right wheel (IN3 / IN4 / ENB)

    Wiring (physical pin → sysfs GPIO):
        IN1 → PIN_IN1, IN2 → PIN_IN2, ENA → PIN_ENA
        IN3 → PIN_IN3, IN4 → PIN_IN4, ENB → PIN_ENB

    If any GPIO export fails, the constructor raises — the caller must not
    start the motors and should let the program exit cleanly.
    """

    def __init__(self) -> None:
        for pin in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB):
            _export(pin)

        self._pwm_a = _SoftPWM(PIN_ENA, PWM_FREQUENCY_HZ)
        self._pwm_b = _SoftPWM(PIN_ENB, PWM_FREQUENCY_HZ)
        self._pwm_a.start()
        self._pwm_b.start()

        # always start in a safe state
        self.stop()

    def drive_forward(self, speed_pct: float = 100) -> None:
        """Drive both motors forward at speed_pct (0–100)."""
        # IN1=H, IN2=L → Motor A spins forward
        _set(PIN_IN1, 1)
        _set(PIN_IN2, 0)
        # IN3=H, IN4=L → Motor B spins forward
        _set(PIN_IN3, 1)
        _set(PIN_IN4, 0)
        self._pwm_a.set_duty(speed_pct)
        self._pwm_b.set_duty(speed_pct)

    def stop(self) -> None:
        """Cut power to both motors immediately."""
        self._pwm_a.set_duty(0)
        self._pwm_b.set_duty(0)
        for pin in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4):
            _set(pin, 0)

    # ── GPS phase stubs (kept so gps/navigator.py can still call them) ─────
    def turn_left(self, speed_pct: float = 40) -> None:
        # TODO (phase 2): left motor backward, right motor forward
        self.stop()

    def turn_right(self, speed_pct: float = 40) -> None:
        # TODO (phase 2): left motor forward, right motor backward
        self.stop()
    # ────────────────────────────────────────────────────────────────────────

    def close(self) -> None:
        """Stop motors, kill PWM threads, and release sysfs exports."""
        self.stop()
        self._pwm_a.stop()
        self._pwm_b.stop()
        for pin in (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB):
            _unexport(pin)
