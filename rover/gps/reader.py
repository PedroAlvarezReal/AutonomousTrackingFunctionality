import serial
import pynmea2
from config import GPS_PORT, GPS_BAUD


class GPSReader:
    def __init__(self):
        print(f"Connecting to GPS on {GPS_PORT}...")
        self.ser = serial.Serial(GPS_PORT, baudrate=GPS_BAUD, timeout=1)
        self.lat = None
        self.lon = None

    def update(self):
        """
        Read one line from GPS. Call this in a loop.
        Returns True if we got a valid fix, False otherwise.
        """
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()

            if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                msg = pynmea2.parse(line)

                if msg.status == 'A':  # A = active fix
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    return True

        except pynmea2.ParseError:
            pass
        except Exception as e:
            print(f"GPS read error: {e}")

        return False

    def get_position(self):
        """Returns (lat, lon) or (None, None) if no fix yet."""
        return self.lat, self.lon

    def close(self):
        self.ser.close()
