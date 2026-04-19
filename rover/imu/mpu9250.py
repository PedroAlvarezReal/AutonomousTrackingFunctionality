"""MPU-9250 magnetometer heading reader.

Uses the AK8963 compass inside the MPU-9250 over I2C bypass mode.
This gives the rover a real heading while stationary, unlike GPS-only
course-over-ground which only becomes useful after the rover has moved.
"""

from __future__ import annotations

import math
import time

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - fallback when smbus2 is unavailable
    from smbus import SMBus  # type: ignore

from config import (
    AK8963_ADDRESS,
    HEADING_DECLINATION_DEG,
    HEADING_OFFSET_DEG,
    HEADING_SMOOTHING_ALPHA,
    MPU9250_ADDRESS,
    MPU9250_I2C_BUS,
    MPU9250_MAG_OFFSET_X,
    MPU9250_MAG_OFFSET_Y,
    MPU9250_MAG_OFFSET_Z,
    MPU9250_MAG_SCALE_X,
    MPU9250_MAG_SCALE_Y,
    MPU9250_MAG_SCALE_Z,
)


class MPU9250Compass:
    """Read rover heading from the MPU-9250 magnetometer."""

    MPU_PWR_MGMT_1 = 0x6B
    MPU_USER_CTRL = 0x6A
    MPU_INT_PIN_CFG = 0x37
    MPU_WHO_AM_I = 0x75

    AK_WHO_AM_I = 0x00
    AK_ST1 = 0x02
    AK_HXL = 0x03
    AK_ST2 = 0x09
    AK_CNTL1 = 0x0A
    AK_CNTL2 = 0x0B
    AK_ASAX = 0x10

    def __init__(self) -> None:
        self._bus = SMBus(MPU9250_I2C_BUS)
        self.address = None
        self._asa = (1.0, 1.0, 1.0)
        self._heading = None
        self._configure()

    def _write_mpu(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self.address, reg, value)

    def _read_mpu(self, reg: int) -> int:
        return self._bus.read_byte_data(self.address, reg)

    def _write_mag(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(AK8963_ADDRESS, reg, value)

    def _read_mag(self, reg: int) -> int:
        return self._bus.read_byte_data(AK8963_ADDRESS, reg)

    def _read_mag_block(self, reg: int, length: int) -> list[int]:
        return self._bus.read_i2c_block_data(AK8963_ADDRESS, reg, length)

    @staticmethod
    def _to_signed(lo: int, hi: int) -> int:
        value = (hi << 8) | lo
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def _wrap_heading(deg: float) -> float:
        return deg % 360.0

    @staticmethod
    def _blend_heading(prev_deg: float, new_deg: float, alpha: float) -> float:
        prev_rad = math.radians(prev_deg)
        new_rad = math.radians(new_deg)
        x = (1.0 - alpha) * math.cos(prev_rad) + alpha * math.cos(new_rad)
        y = (1.0 - alpha) * math.sin(prev_rad) + alpha * math.sin(new_rad)
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            return new_deg
        return math.degrees(math.atan2(y, x)) % 360.0

    def _configure(self) -> None:
        self.address = self._detect_mpu_address()

        # Wake the MPU-9250 and put auxiliary I2C into pass-through mode.
        self._write_mpu(self.MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self._write_mpu(self.MPU_USER_CTRL, 0x00)
        self._write_mpu(self.MPU_INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        mpu_id = self._read_mpu(self.MPU_WHO_AM_I)
        if mpu_id not in (0x71, 0x73):
            raise RuntimeError(f"Unexpected MPU-9250 WHO_AM_I value: 0x{mpu_id:02X}")

        mag_id = self._read_mag(self.AK_WHO_AM_I)
        if mag_id != 0x48:
            raise RuntimeError(
                f"MPU-9250 responded at 0x{self.address:02X}, but AK8963 magnetometer at 0x{AK8963_ADDRESS:02X} "
                f"did not respond correctly (WHO_AM_I=0x{mag_id:02X}). Check the module wiring and make sure "
                f"the breakout really exposes the MPU-9250 compass."
            )

        # Power down before switching AK8963 modes.
        self._write_mag(self.AK_CNTL1, 0x00)
        time.sleep(0.01)

        # Read factory sensitivity adjustment values from fuse ROM.
        self._write_mag(self.AK_CNTL1, 0x0F)
        time.sleep(0.01)
        asa = self._read_mag_block(self.AK_ASAX, 3)
        self._asa = tuple((((value - 128) * 0.5) / 128.0) + 1.0 for value in asa)

        # Return to power down, then start 16-bit continuous mode 2 (100 Hz).
        self._write_mag(self.AK_CNTL1, 0x00)
        time.sleep(0.01)
        self._write_mag(self.AK_CNTL1, 0x16)
        time.sleep(0.01)

    def _detect_mpu_address(self) -> int:
        candidates = []
        for address in (MPU9250_ADDRESS, 0x69, 0x68):
            if address not in candidates:
                candidates.append(address)

        for address in candidates:
            try:
                who_am_i = self._bus.read_byte_data(address, self.MPU_WHO_AM_I)
            except OSError:
                continue
            if who_am_i in (0x71, 0x73):
                return address
            raise RuntimeError(
                f"I2C device answered at 0x{address:02X}, but WHO_AM_I was 0x{who_am_i:02X} instead of 0x71/0x73."
            )

        tried = ", ".join(f"0x{address:02X}" for address in candidates)
        raise RuntimeError(
            f"No MPU-9250 found on /dev/i2c-{MPU9250_I2C_BUS}. Tried addresses {tried}. "
            f"Run `i2cdetect -a -y -r {MPU9250_I2C_BUS}` on the rover to see what is actually present."
        )

    def read_heading(self) -> float | None:
        """Return smoothed compass heading in degrees, or None if not ready."""
        st1 = self._read_mag(self.AK_ST1)
        if not (st1 & 0x01):
            return None

        raw = self._read_mag_block(self.AK_HXL, 7)
        st2 = raw[6]
        if st2 & 0x08:
            return None

        x = self._to_signed(raw[0], raw[1]) * self._asa[0]
        y = self._to_signed(raw[2], raw[3]) * self._asa[1]
        z = self._to_signed(raw[4], raw[5]) * self._asa[2]

        # Hard-iron and soft-iron correction hooks.
        x = (x - MPU9250_MAG_OFFSET_X) * MPU9250_MAG_SCALE_X
        y = (y - MPU9250_MAG_OFFSET_Y) * MPU9250_MAG_SCALE_Y
        z = (z - MPU9250_MAG_OFFSET_Z) * MPU9250_MAG_SCALE_Z

        del z  # the current rover logic uses a level heading only

        heading = math.degrees(math.atan2(y, x))
        heading = self._wrap_heading(heading + HEADING_DECLINATION_DEG + HEADING_OFFSET_DEG)

        if self._heading is None:
            self._heading = heading
        else:
            self._heading = self._blend_heading(
                self._heading,
                heading,
                max(0.0, min(1.0, HEADING_SMOOTHING_ALPHA)),
            )
        return self._heading

    def close(self) -> None:
        try:
            self._write_mag(self.AK_CNTL1, 0x00)
        except Exception:
            pass
        try:
            self._bus.close()
        except Exception:
            pass
