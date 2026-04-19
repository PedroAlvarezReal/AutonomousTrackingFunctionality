#!/usr/bin/env python3
"""Quick MPU-9250 heading test."""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from imu.mpu9250 import MPU9250Heading


def main() -> None:
    compass = MPU9250Heading()
    print(
        f"Heading IMU: {compass.core_name} "
        f"(WHO_AM_I=0x{compass.core_id:02X}, /dev/i2c-{compass.bus_id}, "
        f"addr=0x{compass.address:02X}, mode={compass.mode}, {compass.mag_status})"
    )
    if compass.mode != "magnetometer":
        compass.seed_heading(0.0, absolute=False)
        print("Gyro fallback mode: heading starts at 0 deg and tracks turns from there.")
        print("GPS motion will be needed later if you want true world heading.")
    print("Starting heading test. Press Ctrl+C to stop.")
    try:
        while True:
            heading = compass.read_heading()
            if heading is not None:
                print(
                    f"\rheading={heading:7.2f} deg  source={compass.nav_source:8s}  "
                    f"gyro_z={compass.last_rate_dps:7.2f} dps",
                    end="",
                    flush=True,
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        compass.close()


if __name__ == "__main__":
    main()
