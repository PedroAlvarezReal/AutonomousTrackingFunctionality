# ── rover/motors/controller.py ───────────────────────────────────────────────
# Motor control via serial commands to an Arduino Uno.
#
# Architecture:
#   Rubik Pi 3  ──USB──►  Arduino Uno  ──L298N──►  Motors
#
# The Pi handles ALL sensing (camera, ultrasonic) and decision-making.
# It sends simple text commands over serial to the Arduino, which only
# does one job: translate commands into L298N H-bridge pin signals.
#
# Serial protocol (Pi → Arduino):
#   "F<speed>\n"   drive Forward at speed (0–100)
#   "B<speed>\n"   drive Backward at speed (0–100)
#   "L<speed>\n"   turn Left at speed (0–100)
#   "R<speed>\n"   turn Right at speed (0–100)
#   "S\n"          Stop immediately
#
# The Arduino echoes "OK\n" after each command (optional, not waited on).

import glob
import os
import serial
import threading
import time

from config import ARDUINO_SERIAL_PORT, ARDUINO_BAUD_RATE


class MotorController:
    """
    Sends motor commands to an Arduino Uno over serial USB.

    The Arduino runs a sketch that reads these commands and drives
    the L298N H-bridge accordingly.  See arduino/motor_driver.ino.
    """

    @staticmethod
    def _resolve_port() -> str:
        """
        Prefer the configured serial port, but fall back to common Arduino
        USB device paths when Linux enumerates the adapter differently.
        """
        if ARDUINO_SERIAL_PORT and os.path.exists(ARDUINO_SERIAL_PORT):
            return ARDUINO_SERIAL_PORT

        candidates = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
        if candidates:
            return candidates[0]

        raise serial.SerialException(
            f"Arduino serial port not found. Tried {ARDUINO_SERIAL_PORT!r}, /dev/ttyUSB*, /dev/ttyACM*"
        )

    def __init__(self) -> None:
        port = self._resolve_port()
        self._ser = serial.Serial(
            port=port,
            baudrate=ARDUINO_BAUD_RATE,
            timeout=1.0,
        )
        self.port = port
        self._lock = threading.Lock()
        # Arduino resets when USB-serial connects — wait for it to boot
        time.sleep(2.0)
        self._ser.reset_input_buffer()
        # Start in a safe state
        self.stop()

    def _send(self, cmd: str) -> None:
        """Send a command string to the Arduino."""
        with self._lock:
            self._ser.write(f"{cmd}\n".encode())

    def send_command(self, cmd: str) -> None:
        """Public wrapper for sending arbitrary commands (e.g. servo)."""
        self._send(cmd)

    def drive_forward(self, speed_pct: float = 100) -> None:
        """Drive both motors forward at speed_pct (0–100)."""
        speed = int(max(0, min(100, speed_pct)))
        self._send(f"F{speed}")

    def drive_backward(self, speed_pct: float = 100) -> None:
        """Drive both motors backward at speed_pct (0–100)."""
        speed = int(max(0, min(100, speed_pct)))
        self._send(f"B{speed}")

    def stop(self) -> None:
        """Cut power to both motors immediately."""
        self._send("S")

    def turn_left(self, speed_pct: float = 40) -> None:
        """Left motor backward, right motor forward."""
        speed = int(max(0, min(100, speed_pct)))
        self._send(f"L{speed}")

    def turn_right(self, speed_pct: float = 40) -> None:
        """Left motor forward, right motor backward."""
        speed = int(max(0, min(100, speed_pct)))
        self._send(f"R{speed}")

    def close(self) -> None:
        """Stop motors and close serial connection."""
        try:
            self.stop()
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass
