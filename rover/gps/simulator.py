import math
import time


class GPSSimulator:
    """
    Fake GPS that moves toward a target.
    Use this to test navigation logic without real hardware.
    """

    def __init__(self, start_lat, start_lon, speed_mps=1.0):
        self.lat = start_lat
        self.lon = start_lon
        self.speed = speed_mps
        self.heading = 0.0
        self._last_time = time.time()

    def move(self, bearing_deg):
        """Move one step toward a bearing. Call this each loop iteration."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        distance = self.speed * dt
        R = 6371000
        bearing = math.radians(bearing_deg)
        lat1 = math.radians(self.lat)
        lon1 = math.radians(self.lon)

        lat2 = math.asin(
            math.sin(lat1) * math.cos(distance / R) +
            math.cos(lat1) * math.sin(distance / R) * math.cos(bearing)
        )
        lon2 = lon1 + math.atan2(
            math.sin(bearing) * math.sin(distance / R) * math.cos(lat1),
            math.cos(distance / R) - math.sin(lat1) * math.sin(lat2)
        )

        self.lat = math.degrees(lat2)
        self.lon = math.degrees(lon2)
        self.heading = bearing_deg

    def get_position(self):
        return self.lat, self.lon

    def update(self):
        """Matches GPSReader interface so main.py works with both."""
        return True
