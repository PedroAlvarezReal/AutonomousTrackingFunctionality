"""MPU-9250 / MPU-6500 heading helper.

This module prefers the AK8963 magnetometer when it is reachable through
the MPU-9250 bypass bus. If the board only exposes the core IMU at 0x68/0x69,
it falls back to gyro-integrated yaw that can be seeded and corrected from
GPS course-over-ground while the rover is moving.
"""

from __future__ import annotations

import glob
import math
import os
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
    MPU9250_GYRO_AXIS,
    MPU9250_GPS_CORRECTION_ALPHA,
    MPU9250_GYRO_BIAS_SAMPLES,
    MPU9250_GYRO_DEADBAND_DPS,
    MPU9250_GYRO_SIGN,
    MPU9250_I2C_BUS,
    MPU9250_I2C_BUS_CANDIDATES,
    MPU9250_MAG_OFFSET_X,
    MPU9250_MAG_OFFSET_Y,
    MPU9250_MAG_OFFSET_Z,
    MPU9250_MAG_SCALE_X,
    MPU9250_MAG_SCALE_Y,
    MPU9250_MAG_SCALE_Z,
)


class MPU9250Heading:
    """Read rover heading from a real magnetometer or gyro fallback."""

    MPU_SMPLRT_DIV = 0x19
    MPU_CONFIG = 0x1A
    MPU_GYRO_CONFIG = 0x1B
    MPU_PWR_MGMT_1 = 0x6B
    MPU_PWR_MGMT_2 = 0x6C
    MPU_USER_CTRL = 0x6A
    MPU_INT_PIN_CFG = 0x37
    MPU_WHO_AM_I = 0x75
    GYRO_AXIS_REGS = {
        "x": 0x43,
        "y": 0x45,
        "z": 0x47,
    }

    AK_WHO_AM_I = 0x00
    AK_ST1 = 0x02
    AK_HXL = 0x03
    AK_ST2 = 0x09
    AK_CNTL1 = 0x0A
    AK_ASAX = 0x10

    VALID_CORE_IDS = {
        0x70: "MPU-6500 / MPU-9250 clone",
        0x71: "MPU-9250",
        0x73: "MPU-9255 / compatible",
    }

    GYRO_SENSITIVITY_LSB_PER_DPS = 131.0  # +-250 dps full scale

    def __init__(self) -> None:
        self.bus_id: int | None = None
        self._bus: SMBus | None = None
        self.address: int | None = None
        self.core_id: int | None = None
        self.core_name = "unknown"
        self.mode = "gyro+gps"
        self.mag_status = "not probed"
        self._mag_available = False
        self._absolute_heading = False
        self._heading: float | None = None
        self._asa = (1.0, 1.0, 1.0)
        self._gyro_bias_dps = 0.0
        self._gyro_last_rate_dps = 0.0
        self._last_gyro_ts: float | None = None
        self.gyro_axis_name = "z"
        self._gyro_axis_reg = self.GYRO_AXIS_REGS["z"]
        self._configure()

    @property
    def nav_source(self) -> str:
        if self._heading is None:
            return "unknown"
        return "imu" if self._absolute_heading else "imu-init"

    @property
    def last_rate_dps(self) -> float:
        return self._gyro_last_rate_dps

    @property
    def gyro_bias_dps(self) -> float:
        return self._gyro_bias_dps

    def _write_mpu(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self.address, reg, value)

    def _read_mpu(self, reg: int) -> int:
        return self._bus.read_byte_data(self.address, reg)

    def _read_mpu_block(self, reg: int, length: int) -> list[int]:
        return self._bus.read_i2c_block_data(self.address, reg, length)

    def _write_mag(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(AK8963_ADDRESS, reg, value)

    def _read_mag(self, reg: int) -> int:
        return self._bus.read_byte_data(AK8963_ADDRESS, reg)

    def _read_mag_block(self, reg: int, length: int) -> list[int]:
        return self._bus.read_i2c_block_data(AK8963_ADDRESS, reg, length)

    @staticmethod
    def _to_signed(hi: int, lo: int) -> int:
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
        self.bus_id, self._bus, self.address = self._detect_mpu()
        self.gyro_axis_name = str(MPU9250_GYRO_AXIS).strip().lower()
        if self.gyro_axis_name not in self.GYRO_AXIS_REGS:
            raise RuntimeError(
                f"Invalid MPU9250_GYRO_AXIS={MPU9250_GYRO_AXIS!r}. "
                f"Use one of: {', '.join(sorted(self.GYRO_AXIS_REGS))}."
            )
        self._gyro_axis_reg = self.GYRO_AXIS_REGS[self.gyro_axis_name]

        # Wake the IMU core and configure a steady gyro data stream.
        self._write_mpu(self.MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        self._write_mpu(self.MPU_PWR_MGMT_2, 0x00)
        self._write_mpu(self.MPU_USER_CTRL, 0x00)
        self._write_mpu(self.MPU_INT_PIN_CFG, 0x02)
        self._write_mpu(self.MPU_CONFIG, 0x03)
        self._write_mpu(self.MPU_SMPLRT_DIV, 0x04)
        self._write_mpu(self.MPU_GYRO_CONFIG, 0x00)
        time.sleep(0.02)

        self.core_id = self._read_mpu(self.MPU_WHO_AM_I)
        self.core_name = self.VALID_CORE_IDS.get(self.core_id, "unknown")
        if self.core_id not in self.VALID_CORE_IDS:
            raise RuntimeError(
                f"Unexpected IMU WHO_AM_I value: 0x{self.core_id:02X}. "
                f"Expected one of {', '.join(f'0x{k:02X}' for k in sorted(self.VALID_CORE_IDS))}."
            )

        self._gyro_bias_dps = self._calibrate_gyro_bias()
        self._last_gyro_ts = time.monotonic()

        mag_id, mag_error = self._probe_mag_id()
        if mag_id == 0x48:
            self._configure_mag()
            self.mode = "magnetometer"
            self.mag_status = "AK8963 detected"
            self._mag_available = True
            self._absolute_heading = True
        else:
            if mag_id is None:
                self.mag_status = f"AK8963 unavailable ({mag_error})"
            else:
                self.mag_status = f"AK8963 unavailable (WHO_AM_I=0x{mag_id:02X})"
            self.mode = "gyro+gps"
            self._mag_available = False
            self._absolute_heading = False

    def _configure_mag(self) -> None:
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

    def _probe_mag_id(self) -> tuple[int | None, str | None]:
        last_error = None
        for _ in range(8):
            try:
                return self._read_mag(self.AK_WHO_AM_I), None
            except OSError as exc:
                last_error = exc
                time.sleep(0.02)
        if last_error is not None:
            return None, str(last_error)
        return None, "unknown error"

    def _candidate_buses(self) -> list[int]:
        candidates = []
        for bus_id in MPU9250_I2C_BUS_CANDIDATES:
            if bus_id not in candidates:
                candidates.append(bus_id)
        if MPU9250_I2C_BUS not in candidates:
            candidates.insert(0, MPU9250_I2C_BUS)

        for path in sorted(glob.glob("/dev/i2c-*")):
            try:
                bus_id = int(path.rsplit("-", 1)[1])
            except ValueError:
                continue
            if bus_id not in candidates:
                candidates.append(bus_id)
        return candidates

    def _detect_mpu(self) -> tuple[int, SMBus, int]:
        candidates = []
        for address in (MPU9250_ADDRESS, 0x69, 0x68):
            if address not in candidates:
                candidates.append(address)

        last_error = None
        tried_pairs = []
        for bus_id in self._candidate_buses():
            if not os.path.exists(f"/dev/i2c-{bus_id}"):
                continue
            try:
                bus = SMBus(bus_id)
            except Exception as exc:
                last_error = exc
                continue

            for address in candidates:
                tried_pairs.append(f"/dev/i2c-{bus_id}@0x{address:02X}")
                try:
                    who_am_i = bus.read_byte_data(address, self.MPU_WHO_AM_I)
                except OSError as exc:
                    last_error = exc
                    continue
                if who_am_i in self.VALID_CORE_IDS:
                    return bus_id, bus, address
                bus.close()
                raise RuntimeError(
                    f"I2C device answered on /dev/i2c-{bus_id} at 0x{address:02X}, "
                    f"but WHO_AM_I was 0x{who_am_i:02X} instead of one of "
                    f"{', '.join(f'0x{k:02X}' for k in sorted(self.VALID_CORE_IDS))}."
                )

            bus.close()

        tried = ", ".join(tried_pairs) if tried_pairs else f"/dev/i2c-{MPU9250_I2C_BUS}@0x68/0x69"
        extra = f" Last error: {last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"No MPU-9250 found. Tried {tried}.{extra} "
            f"Run `i2cdetect -a -y -r 0`, `1`, and `9` on the rover to see what is actually present."
        )

    def _read_gyro_dps(self) -> float:
        raw = self._read_mpu_block(self._gyro_axis_reg, 2)
        rate = self._to_signed(raw[0], raw[1]) / self.GYRO_SENSITIVITY_LSB_PER_DPS
        rate *= MPU9250_GYRO_SIGN
        rate -= self._gyro_bias_dps
        if abs(rate) < MPU9250_GYRO_DEADBAND_DPS:
            rate = 0.0
        self._gyro_last_rate_dps = rate
        return rate

    def _calibrate_gyro_bias(self) -> float:
        samples = max(50, int(MPU9250_GYRO_BIAS_SAMPLES))
        total = 0.0
        count = 0
        for _ in range(samples):
            raw = self._read_mpu_block(self._gyro_axis_reg, 2)
            rate = self._to_signed(raw[0], raw[1]) / self.GYRO_SENSITIVITY_LSB_PER_DPS
            rate *= MPU9250_GYRO_SIGN
            total += rate
            count += 1
            time.sleep(0.004)
        if count == 0:
            raise RuntimeError(f"Unable to read gyro {self.gyro_axis_name.upper()} axis for startup calibration.")
        return total / count

    def seed_heading(self, heading_deg: float, *, absolute: bool = False) -> float:
        """Set the current rover heading.

        Use `absolute=True` when the value comes from a real world reference,
        such as GPS course-over-ground. Use `absolute=False` for bootstrap
        estimates, for example the initial target bearing before GPS has
        observed enough motion to know true course yet.
        """

        self._heading = self._wrap_heading(heading_deg)
        self._absolute_heading = absolute or self._mag_available
        self._last_gyro_ts = time.monotonic()
        return self._heading

    def correct_heading(self, heading_deg: float, alpha: float | None = None) -> float:
        """Blend an external absolute heading into the current estimate."""

        alpha = MPU9250_GPS_CORRECTION_ALPHA if alpha is None else alpha
        alpha = max(0.0, min(1.0, alpha))
        heading_deg = self._wrap_heading(heading_deg)

        if self._heading is None or not self._absolute_heading:
            return self.seed_heading(heading_deg, absolute=True)

        self._heading = self._blend_heading(self._heading, heading_deg, alpha)
        self._absolute_heading = True
        self._last_gyro_ts = time.monotonic()
        return self._heading

    def _read_mag_heading(self) -> float | None:
        st1 = self._read_mag(self.AK_ST1)
        if not (st1 & 0x01):
            return None

        raw = self._read_mag_block(self.AK_HXL, 7)
        st2 = raw[6]
        if st2 & 0x08:
            return None

        x = self._to_signed(raw[1], raw[0]) * self._asa[0]
        y = self._to_signed(raw[3], raw[2]) * self._asa[1]
        z = self._to_signed(raw[5], raw[4]) * self._asa[2]

        x = (x - MPU9250_MAG_OFFSET_X) * MPU9250_MAG_SCALE_X
        y = (y - MPU9250_MAG_OFFSET_Y) * MPU9250_MAG_SCALE_Y
        z = (z - MPU9250_MAG_OFFSET_Z) * MPU9250_MAG_SCALE_Z

        del z

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
        self._absolute_heading = True
        self._last_gyro_ts = time.monotonic()
        return self._heading

    def _read_gyro_heading(self) -> float | None:
        now = time.monotonic()
        rate = self._read_gyro_dps()
        if self._last_gyro_ts is None:
            self._last_gyro_ts = now
            return self._heading

        dt = max(0.0, min(now - self._last_gyro_ts, 0.25))
        self._last_gyro_ts = now
        if self._heading is None:
            return None

        self._heading = self._wrap_heading(self._heading + (rate * dt))
        return self._heading

    def read_heading(self) -> float | None:
        """Return current heading in degrees, or None if not initialized yet."""

        if self._mag_available:
            try:
                heading = self._read_mag_heading()
                if heading is not None:
                    return heading
            except Exception:
                # If the magnetometer flakes out, keep the last heading alive
                # by integrating gyro yaw until the next good absolute sample.
                pass
        return self._read_gyro_heading()

    def close(self) -> None:
        if self._mag_available:
            try:
                self._write_mag(self.AK_CNTL1, 0x00)
            except Exception:
                pass
        try:
            self._bus.close()
        except Exception:
            pass


# Backward-compatible name used elsewhere in the project.
MPU9250Compass = MPU9250Heading
