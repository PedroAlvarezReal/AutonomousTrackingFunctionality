#!/usr/bin/env python3
# ── rover/test_gps_reader.py ─────────────────────────────────────────────────
# Clean GPS live dashboard — no pynmea2 needed, parses NMEA directly.
# Clears screen each second and shows a tidy coordinate readout.
#
# Usage:
#   sudo /home/ubuntu/AutonomousTrackingFunctionality/venv/bin/python3 test_gps_reader.py

import sys
import os
import time
import serial

sys.path.insert(0, os.path.dirname(__file__))

GPS_PORT = '/dev/ttyHS2'
GPS_BAUD = 9600

GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
CLEAR  = "\033c"


def _nmea_dms_to_dd(raw, hemi):
    """Convert NMEA DDDMM.MMMMM + hemisphere to decimal degrees."""
    if not raw:
        return None
    dot = raw.index('.')
    deg = float(raw[:dot - 2])
    mins = float(raw[dot - 2:])
    dd = deg + mins / 60.0
    if hemi in ('S', 'W'):
        dd = -dd
    return dd


def _parse_rmc(fields):
    """$GPRMC — lat/lon/speed/course/status."""
    if len(fields) < 10:
        return None
    status = fields[2]
    if status != 'A':
        return {'fix': False}
    lat = _nmea_dms_to_dd(fields[3], fields[4])
    lon = _nmea_dms_to_dd(fields[5], fields[6])
    speed_kn = float(fields[7]) if fields[7] else 0.0
    course   = float(fields[8]) if fields[8] else None
    utc_raw  = fields[1]   # HHMMSS.ss
    return {
        'fix': True,
        'lat': lat,
        'lon': lon,
        'speed_kph': speed_kn * 1.852,
        'course': course,
        'utc': utc_raw,
    }


def _parse_gga(fields):
    """$GPGGA — fix quality, satellites used, altitude, HDOP."""
    if len(fields) < 15:
        return None
    qual = int(fields[6]) if fields[6] else 0
    sats = int(fields[7]) if fields[7] else 0
    hdop = float(fields[8]) if fields[8] else None
    alt  = float(fields[9]) if fields[9] else None
    return {'qual': qual, 'sats': sats, 'hdop': hdop, 'alt': alt}


def _parse_gsv(fields):
    """$GPGSV — satellites in view (first message only)."""
    if len(fields) < 4:
        return None
    if fields[1] != '1':   # only read msg 1 of N
        return None
    return {'siv': int(fields[3]) if fields[3] else 0}


def _draw(state, elapsed):
    fix   = state.get('fix', False)
    lat   = state.get('lat')
    lon   = state.get('lon')
    qual  = state.get('qual', 0)
    sats  = state.get('sats', 0)
    siv   = state.get('siv', 0)
    hdop  = state.get('hdop')
    alt   = state.get('alt')
    spd   = state.get('speed_kph', 0.0)
    crs   = state.get('course')
    utc   = state.get('utc', '')
    count = state.get('count', 0)

    # Format UTC
    if len(utc) >= 6:
        utc_str = f"{utc[0:2]}:{utc[2:4]}:{utc[4:6]} UTC"
    else:
        utc_str = '--:--:-- UTC'

    qual_label = {0: 'No fix', 1: 'GPS fix', 2: 'DGPS fix'}.get(qual, f'Quality {qual}')
    fix_col    = GREEN if fix and qual > 0 else YELLOW

    print(CLEAR, end='')
    print(f"{BOLD}{'=' * 50}{RESET}")
    print(f"{BOLD}  GPS Live Dashboard{RESET}   {DIM}(Ctrl+C to stop){RESET}")
    print(f"{'=' * 50}")
    print()

    if fix and lat is not None:
        print(f"  {BOLD}Status   {RESET}  {fix_col}{qual_label}{RESET}")
        print(f"  {BOLD}Latitude {RESET}  {GREEN}{lat:+.7f}{RESET}")
        print(f"  {BOLD}Longitude{RESET}  {GREEN}{lon:+.7f}{RESET}")
        print()
        print(f"  {BOLD}Satellites{RESET}  {sats} used / {siv} in view")
        print(f"  {BOLD}HDOP      {RESET}  {hdop:.2f}  {'(good)' if hdop and hdop < 2 else '(fair)' if hdop and hdop < 5 else '(poor)'}")
        print(f"  {BOLD}Altitude  {RESET}  {alt:.1f} m" if alt is not None else f"  {BOLD}Altitude  {RESET}  N/A")
        print(f"  {BOLD}Speed     {RESET}  {spd:.1f} km/h")
        print(f"  {BOLD}Course    {RESET}  {crs:.1f} deg" if crs is not None else f"  {BOLD}Course    {RESET}  --")
        print(f"  {BOLD}Time      {RESET}  {utc_str}")
        print()
        print(f"  {DIM}Google Maps: https://maps.google.com/?q={lat:.7f},{lon:.7f}{RESET}")
    else:
        print(f"  {YELLOW}Waiting for fix...{RESET}  ({elapsed:.0f}s elapsed)")
        print()
        print(f"  Satellites in view: {siv}")
        print()
        print(f"  {DIM}Tips: ensure clear sky view, cold start ~1-3 min{RESET}")

    print()
    print(f"{'=' * 50}")
    print(f"  {DIM}Port: {GPS_PORT}   Updates: {count}{RESET}")


def main():
    try:
        ser = serial.Serial(GPS_PORT, baudrate=GPS_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"{RED}[ERROR] Cannot open {GPS_PORT}: {e}{RESET}")
        sys.exit(1)

    state      = {'fix': False, 'count': 0}
    start_time = time.time()
    last_draw  = 0.0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode('ascii', errors='replace').strip()
            if not line.startswith('$'):
                continue

            parts = line.split('*')[0].split(',')
            tag   = parts[0].upper()

            if tag in ('$GPRMC', '$GNRMC'):
                parsed = _parse_rmc(parts)
                if parsed:
                    state.update(parsed)
                    state['count'] += 1

            elif tag in ('$GPGGA', '$GNGGA'):
                parsed = _parse_gga(parts)
                if parsed:
                    state.update(parsed)

            elif tag in ('$GPGSV', '$GNGSV'):
                parsed = _parse_gsv(parts)
                if parsed:
                    state.update(parsed)

            # Redraw at most once per second
            now = time.time()
            if now - last_draw >= 1.0:
                _draw(state, now - start_time)
                last_draw = now

    except KeyboardInterrupt:
        print(f"\n{BOLD}Stopped.{RESET}")
        if state.get('lat') is not None:
            print(f"Last fix: {state['lat']:+.7f}, {state['lon']:+.7f}")
        ser.close()


if __name__ == "__main__":
    main()
