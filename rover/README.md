# Rover

Simulated GPS rover that navigates to a target coordinate. No hardware needed.

## Setup

```bash
pip3 install pyserial pynmea2
```

Make sure `config.py` has:
```python
SIMULATION_MODE = True
```

## Run

```bash
cd rover
python3 main.py
```

Hit `Ctrl+C` to stop early.

## Change the target

Edit these lines in `main.py`:
```python
TARGET_LAT = 41.8819
TARGET_LON = -87.6290
```

Get coordinates by right-clicking any spot on Google Maps.

## Change start position

Edit this line in `main.py`:
```python
gps = GPSSimulator(start_lat=41.8827, start_lon=-87.6233)
```

## Tune behaviour

All in `config.py`:

| Setting | Default | What it does |
|---|---|---|
| `ARRIVAL_THRESHOLD` | `2.5` | Meters from target before stopping |
| `HEADING_TOLERANCE` | `10` | Degrees off before it turns |
| `DRIVE_SPEED` | `60` | Forward speed (0–100) |
| `TURN_SPEED` | `40` | Turn speed (0–100) |
