import time
from config import (
    ARRIVAL_THRESHOLD,
    HEADING_TOLERANCE,
    SIMULATION_MODE
)
from gps.navigator import get_bearing, get_distance_meters, get_turn_error
from motors.controller import MotorController

# ── Pin your target here ─────────────────────────────────────
TARGET_LAT = 41.8819
TARGET_LON = -87.6290
# ─────────────────────────────────────────────────────────────

def run():
    motors = MotorController()

    # Swap between real GPS and simulator with one flag in config.py
    if SIMULATION_MODE:
        from gps.simulator import GPSSimulator
        gps = GPSSimulator(start_lat=41.8827, start_lon=-87.6233)
        print("Running in SIMULATION MODE")
    else:
        from gps.reader import GPSReader
        gps = GPSReader()
        print("Running with REAL GPS")

    print(f"Target: ({TARGET_LAT}, {TARGET_LON})")
    print(f"Arrival threshold: {ARRIVAL_THRESHOLD}m")
    print("-" * 50)

    current_heading = 0.0  # starts facing North

    try:
        while True:
            # 1 — Get current position
            gps.update()
            curr_lat, curr_lon = gps.get_position()

            if curr_lat is None:
                print("Waiting for GPS fix...")
                time.sleep(1)
                continue

            # 2 — Calculate distance and bearing to target
            distance = get_distance_meters(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)
            bearing  = get_bearing(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)
            error    = get_turn_error(current_heading, bearing)

            print(f"({curr_lat:.6f}, {curr_lon:.6f}) | "
                  f"{distance:.1f}m | "
                  f"{bearing:.1f}° | "
                  f"{error:+.1f}°")

            # 3 — Arrived?
            if distance < ARRIVAL_THRESHOLD:
                motors.stop()
                print("\nArrived at target!")
                break

            # 4 — Need to turn?
            if abs(error) > HEADING_TOLERANCE:
                if error > 0:
                    motors.turn_right()
                else:
                    motors.turn_left()
                current_heading = bearing  # simulate turn completing

            # 5 — Drive forward
            else:
                motors.drive_forward()
                if SIMULATION_MODE:
                    gps.move(bearing)  # advance simulated position

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        motors.stop()

if __name__ == '__main__':
    run()
