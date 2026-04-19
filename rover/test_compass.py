#!/usr/bin/env python3
"""Quick MPU-9250 compass heading test."""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from imu.mpu9250 import MPU9250Compass


def main() -> None:
    compass = MPU9250Compass()
    print("Starting compass heading test. Press Ctrl+C to stop.")
    try:
        while True:
            heading = compass.read_heading()
            if heading is not None:
                print(f"\rheading={heading:7.2f} deg", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        compass.close()


if __name__ == "__main__":
    main()

