# Arkairo — Standalone Scripts

Two Python scripts that run **without ROS2**.

---

## Script 1 — Waypoint Generator

**File:** `waypoint_generator.py`

**Dependencies:** Python 3 stdlib only (zero pip install)

**Workflow:**

1. Draw your field boundary as a polygon in **Google Earth / My Maps**
2. Export it as a `.kml` file
3. Run:

```bash
python waypoint_generator.py path/to/field.kml
```

4. The `.waypoints` file is saved to `drone1_ws/missions/`
5. Open **Mission Planner → Flight Plan → Load WP** → select the file
6. Connect drone → Upload → fly

**Without a KML file** (uses the embedded SOE sports-field polygon):

```bash
python waypoint_generator.py
```

**Config** — edit the `Config` class at the top:

| Setting | Default | Meaning |
| --- | --- | --- |
| `ALTITUDE_M` | 6.7 | Flight altitude in metres AGL |
| `LANE_SPACING_M` | 5.0 | Gap between parallel scan lines |
| `BUFFER_M` | 2.0 | Inward boundary buffer |
| `HOME_LAT / HOME_LON` | SOE gate | Your actual launch point |

---

## Script 2 — Disease Detection Service

**File:** `disease_detection_service.py`

**Dependencies:**

```bash
pip install opencv-python numpy pymavlink
```

**Start the service:**

```bash
python disease_detection_service.py --port COM3 --baud 57600
```

**Stop it:** `Ctrl+C`

**Arguments:**

| Argument | Default | Meaning |
| --- | --- | --- |
| `--port` | COM3 | MAVLink port (same one Mission Planner uses) |
| `--baud` | 57600 | Baud rate |
| `--cam` | 0 | Camera: `0`/`1` for USB webcam, or path/URL |
| `--no-gui` | off | Headless mode (no OpenCV window) |

**What it does while running:**

- Reads your camera at ~5 fps (configurable)
- Gets live GPS from the drone via MAVLink
- Detects yellow plant disease (HSV + vegetation context + shape filter)
- Geotags every new unique hotspot with the drone's GPS position
- Appends to `disease_log_YYYYMMDD.csv` in the same folder
- Shows a 2×2 debug window (close with Q, or use `--no-gui`)

**Output CSV columns:** `Timestamp, Detection_ID, Latitude, Longitude, Altitude_m, GPS_Source, Severity, Severity_Score, Area_px, Pixel_X, Pixel_Y, Status`

**No MAVLink?** If `pymavlink` is not installed or the drone is not connected,
the script still runs — detections are logged with the fallback coordinates
set in `Config.GPS_FALLBACK_LAT / LON`.

---

## Config quick-reference

Edit the `Config` class at the top of each script:

| Setting | Script | Meaning |
| --- | --- | --- |
| `SERIAL_PORT` | detection | COM port (e.g. `COM3`, `/dev/ttyUSB0`, `udp:0.0.0.0:14551`) |
| `CAMERA_SOURCE` | detection | `0` = first USB webcam |
| `TARGET_FPS` | detection | Processing rate (skip frames above this) |
| `SHOW_GUI` | detection | `True` = show OpenCV window |
| `GPS_DEDUP_M` | detection | Suppress re-logging same hotspot within N metres |
| `YELLOW_HSV_MIN/MAX` | detection | Tune for your lighting conditions |
