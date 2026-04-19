#!/usr/bin/env python3
# ── rover/test_gps_reader.py ─────────────────────────────────────────────────
# GPS diagnostic: shows raw NMEA, fix status, satellite count, and coordinates.
#
# Usage:
#   sudo /home/ubuntu/AutonomousTrackingFunctionality/venv/bin/python3 test_gps_reader.py
#
# What to expect:
#   - Before fix: "$GPRMC ... V ..." lines (V = void, no fix yet)
#   - After fix (can take 30–120 s outdoors): lat/lon printed with quality info
#   - GPGSA shows satellite IDs + HDOP (horizontal dilution of precision)
#   - GPGSV shows satellites in view

import sys
import os
import time
import serial

sys.path.insert(0, os.path.dirname(__file__))

try:
    import pynmea2
    HAS_PYNMEA2 = True
except ImportError:
    HAS_PYNMEA2 = False
    print("[WARN] pynmea2 not installed — showing raw NMEA only.")
    print("       Install: pip install pynmea2\n")

GPS_PORT = '/dev/ttyHS2'
GPS_BAUD = 9600

# ANSI colours
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def _col(text, colour):
    return f"{colour}{text}{RESET}"


def main():
    print(f"{BOLD}=== GPS Module Diagnostic ==={RESET}")
    print(f"Port: {GPS_PORT}  Baud: {GPS_BAUD}")
    print("Waiting for NMEA sentences... (Ctrl+C to stop)\n")

    try:
        ser = serial.Serial(GPS_PORT, baudrate=GPS_BAUD, timeout=2)
    except serial.SerialException as e:
        print(_col(f"[ERROR] Cannot open {GPS_PORT}: {e}", RED))
        print("Check: ls /dev/ttyHS* /dev/ttyMSM*")
        sys.exit(1)

    print(_col(f"[OK] Serial port open: {GPS_PORT}", GREEN))
    print("-" * 60)

    fix_count   = 0
    void_count  = 0
    start_time  = time.time()
    last_coords = None
    sats_in_use = 0
    hdop        = None

    try:
        while True:
            raw = ser.readline()
            if not raw:
                elapsed = time.time() - start_time
                print(_col(f"[TIMEOUT] No data in 2 s (elapsed {elapsed:.0f}s) — "
                            "check wiring and module power (LED should blink)", YELLOW))
                continue

            line = raw.decode('ascii', errors='replace').strip()

            # Always print the raw sentence (dim)
            print(f"\033[2m{line}\033[0m")

            if not HAS_PYNMEA2 or not line.startswith('$'):
                continue

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            # ── RMC — recommended minimum (has fix status + coords) ──────────
            if isinstance(msg, pynmea2.types.talker.RMC):
                if msg.status == 'A':
                    fix_count += 1
                    last_coords = (msg.latitude, msg.longitude)
                    t = msg.timestamp
                    print(_col(
                        f"  [FIX #{fix_count}] "
                        f"Lat: {msg.latitude:.7f}  Lon: {msg.longitude:.7f}  "
                        f"Speed: {msg.spd_over_grnd:.1f} kn  "
                        f"Course: {msg.true_course}°  "
                        f"Time: {t}",
                        GREEN
                    ))
                else:
                    void_count += 1
                    elapsed = time.time() - start_time
                    print(_col(
                        f"  [NO FIX] Status=V  (waiting {elapsed:.0f}s — "
                        "move outdoors or near window)",
                        YELLOW
                    ))

            # ── GGA — fix quality + altitude ─────────────────────────────────
            elif isinstance(msg, pynmea2.types.talker.GGA):
                q = int(msg.gps_qual) if msg.gps_qual else 0
                qual_str = {0: "No fix", 1: "GPS fix", 2: "DGPS fix"}.get(q, f"Quality {q}")
                sats = msg.num_sats or '?'
                alt  = f"{msg.altitude} {msg.altitude_units}" if msg.altitude else "N/A"
                print(_col(
                    f"  [GGA] {qual_str}  Sats used: {sats}  Alt: {alt}  "
                    f"HDOP: {msg.horizontal_dil or 'N/A'}",
                    CYAN if q > 0 else YELLOW
                ))

            # ── GSA — satellites + dilution of precision ─────────────────────
            elif isinstance(msg, pynmea2.types.talker.GSA):
                mode   = {"1": "No fix", "2": "2D fix", "3": "3D fix"}.get(msg.mode_fix_type, "?")
                sv_ids = [s for s in [
                              msg.sv_id01, msg.sv_id02, msg.sv_id03,
                              msg.sv_id04, msg.sv_id05, msg.sv_id06,
                              msg.sv_id07, msg.sv_id08, msg.sv_id09,
                              msg.sv_id10, msg.sv_id11, msg.sv_id12,
                          ] if s]
                hdop = msg.hdop
                print(_col(
                    f"  [GSA] {mode}  SVs: {','.join(sv_ids) or 'none'}  "
                    f"PDOP: {msg.pdop}  HDOP: {msg.hdop}  VDOP: {msg.vdop}",
                    CYAN
                ))

            # ── GSV — satellites in view ──────────────────────────────────────
            elif isinstance(msg, pynmea2.types.talker.GSV):
                print(_col(
                    f"  [GSV] Sats in view: {msg.num_sv_in_view}  "
                    f"(msg {msg.msg_num}/{msg.num_messages})",
                    CYAN
                ))

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n{BOLD}=== Summary ==={RESET}")
        print(f"  Runtime       : {elapsed:.0f} s")
        print(f"  Fix sentences : {_col(str(fix_count), GREEN)}")
        print(f"  Void sentences: {_col(str(void_count), YELLOW)}")
        if last_coords:
            lat, lon = last_coords
            print(f"  Last position : {_col(f'{lat:.7f}, {lon:.7f}', GREEN)}")
            print(f"  Google Maps   : https://maps.google.com/?q={lat},{lon}")
        else:
            print(_col("  No valid fix received.", RED))
            print("  Tips:")
            print("    - Move the antenna outdoors or near a window")
            print("    - Cold start can take 1–3 minutes")
            print("    - Check 5 V power to the module (LED should blink ~1 Hz once locked)")
        ser.close()


if __name__ == "__main__":
    main()
