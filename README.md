# tesla-radar-driver

Standalone driver for the Tesla Bosch MRRevo14F radar, with the same API as [toyota-radar-driver](https://github.com/user/toyota-radar-driver) so the same consumer code works with either radar.

## Hardware

- **Radar**: Bosch MRRevo14F (Tesla part, any Model S/X/3 unit)
- **CAN adapters**:
  - Linux/RPi: Waveshare CAN hat or any socketcan adapter
  - macOS: CANalyst-II USB (`canalystii` interface)
- Single CAN bus connection (radar CAN1)

## Quick start

```bash
# Linux (RPi with socketcan)
python3 radar_callbacks.py --channel can0

# macOS (CANalyst-II)
DYLD_LIBRARY_PATH=/opt/homebrew/opt/libusb/lib python3 radar_callbacks.py --channel 0 --interface canalystii

# Curses visualization
python3 radar_curses.py --channel can0
```

## Dependencies

```
python-can
cantools
```

For CANalyst-II on macOS: `pip install canalystii pyusb` and `brew install libusb`.

## API

```python
from tesla_radar_driver import TeslaRadarDriver, TeslaRadarConfig, RadarTrack

config = TeslaRadarConfig(channel="can0")
driver = TeslaRadarDriver(config)

# Register callbacks (same API as ToyotaRadarDriver)
driver.register_track_callback(lambda track: print(track))
driver.start()

# Or poll tracks directly
tracks = driver.get_tracks()  # Dict[int, RadarTrack]
```

### RadarTrack (toyota-radar compatible)

| Field | Type | Description |
|-------|------|-------------|
| `track_id` | int | 0-31 |
| `long_dist` | float | Longitudinal distance (m) |
| `lat_dist` | float | Lateral distance (m) |
| `rel_speed` | float | Relative speed (m/s) |
| `new_track` | int | 1 if first appearance |
| `timestamp` | float | time.time() |
| `raw` | dict | All decoded DBC fields |

### TeslaRadarTrack (extended)

Subclass of `RadarTrack` with typed access to Tesla-specific fields:

| Field | Type | Description |
|-------|------|-------------|
| `prob_exist` | float | Detection probability (0-96.875%) |
| `long_accel` | float | Longitudinal acceleration (m/s^2) |
| `lat_speed` | float | Lateral speed (m/s) |
| `object_class` | int | Classification (0-7) |
| `moving_state` | int | 0-3 |
| `tracked` | int | Tracked flag |
| `valid` | int | Valid flag |
| `meas` | int | Measurement flag |
| `prob_obstacle` | float | Obstacle probability |
| `length` | float | Object length (m) |
| `dz` | float | Height offset (m) |

## Differences from toyota-radar

| Aspect | Toyota | Tesla |
|--------|--------|-------|
| Bus count | 2 (car + radar) | 1 (single bus) |
| Track count | 16 | 32 |
| Track signals | 5 basic | 16+ with classification |
| Keepalive | ACC_CONTROL + 9 static | ~30 vehicle state msgs at 100Hz |
| VIN | Not needed | Auto-read via UDS at startup |

## Files

| File | Description |
|------|-------------|
| `tesla_radar_driver.py` | Main driver (same API as toyota_radar_driver.py) |
| `tesla_radar_protocol.py` | Keepalive protocol (~30 CAN message types at 100Hz) |
| `uds_can.py` | UDS/ISO-TP for VIN auto-read |
| `radar_callbacks.py` | Example: callback logging |
| `radar_curses.py` | Example: curses visualization |
| `opendbc/tesla_radar.dbc` | Tesla radar DBC |
| `opendbc/tesla_can.dbc` | Tesla vehicle CAN DBC (for keepalive encoding) |
