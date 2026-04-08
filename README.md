# Arkairo — Standalone Scripts

> **No ROS2 required.** Two self-contained Python scripts that demonstrate the core drone-agriculture pipeline on any laptop.

---

## What is this?

**Arkairo** is an autonomous agricultural drone system built on ROS2 + ArduPilot.  
This folder contains two **standalone scripts** that replicate the core logic without needing a ROS2 workspace, so you can demo, test, and iterate immediately.

| Script | What it does | Dependencies |
|---|---|---|
| `waypoint_generator.py` | Parses a KML field boundary → generates an ArduPilot `.waypoints` lawnmower mission | **stdlib only** (zero pip install) |
| `disease_detection_service.py` | Reads camera + live drone GPS → detects yellow plant disease → geotags & logs to CSV | `opencv-python numpy pymavlink` |

---

## Quick Start

### 1 — Waypoint Generator

![Waypoints Preview](images/waypoints%20test.jpeg)

```bash
# Use the embedded SOE sports-field demo polygon (no file needed):
python waypoint_generator.py

# Drop your KML file into the missions/ folder and run:
python waypoint_generator.py missions/your_field.kml
```

The generated `.waypoints` file is saved to `missions/` automatically.  
Open it in **Mission Planner → Flight Plan → Load WP**, connect your drone, and upload.

> **Where to put your KML:** Drop it in `missions/` and pass the path as above, or pass any absolute path — the script accepts both.

**Config** — edit the `Config` class at the top of `waypoint_generator.py`:

| Setting | Default | Meaning |
|---|---|---|
| `ALTITUDE_M` | 6.7 | Flight altitude (metres AGL) |
| `LANE_SPACING_M` | 5.0 | Gap between parallel scan lines |
| `BUFFER_M` | 2.0 | Inward boundary buffer |
| `HOME_LAT / HOME_LON` | SOE campus gate | Your actual launch point |

---

### 2 — Disease Detection Service

![Disease Detection Preview](images/detect%20and%20geotag.png)

```bash
pip install opencv-python numpy pymavlink

python disease_detection_service.py --port COM3 --baud 57600
```

Stop with `Ctrl+C`.

**CLI arguments:**

| Argument | Default | Meaning |
|---|---|---|
| `--port` | COM3 | MAVLink serial port (same one Mission Planner uses) |
| `--baud` | 57600 | Baud rate |
| `--cam` | 0 | Camera: `0`/`1` for USB webcam, path or RTSP URL |
| `--no-gui` | off | Headless mode — disables OpenCV window |

**What it does while running:**

- Reads camera at ~5 fps (configurable)
- Pulls live GPS from drone via MAVLink on a background thread
- Detects yellow plant disease (HSV colour filter → vegetation context → shape validation)
- Geotags every new unique hotspot with drone GPS (or fallback coords)
- Appends to `disease_log_YYYYMMDD.csv`
- Shows a live 2×2 debug window (annotated feed | yellow mask | green mask | overlay)

**Output CSV columns:**  
`Timestamp, Detection_ID, Latitude, Longitude, Altitude_m, GPS_Source, Severity, Severity_Score, Area_px, Pixel_X, Pixel_Y, Status`

**No drone connected?** If `pymavlink` is absent or the port fails, the script still runs — detections are logged using the fallback coordinates in `Config.GPS_FALLBACK_LAT / LON`.

---

## Config Quick-Reference

Both scripts expose a `Config` class at the top. Key fields for `disease_detection_service.py`:

| Setting | Meaning |
|---|---|
| `SERIAL_PORT` | MAVLink port (`COM3`, `/dev/ttyUSB0`, `udp:0.0.0.0:14551`) |
| `CAMERA_SOURCE` | `0` = first USB webcam |
| `TARGET_FPS` | Processing rate — higher values use more CPU |
| `SHOW_GUI` | `True` = live OpenCV debug window |
| `GPS_DEDUP_M` | Suppress re-logging the same hotspot within N metres |
| `YELLOW_HSV_MIN/MAX` | Tune for your lighting / crop conditions |
| `HFOV_DEG / VFOV_DEG` | Camera field-of-view for GPS pixel-to-ground projection |

---

## How These Scripts Map to the ROS2 Architecture

```
ROS2 Node (full system)          →   Standalone equivalent
─────────────────────────────────────────────────────────────────
waypoint_generator_node.py       →   waypoint_generator.py
  Publishes: /mission/waypoints       (writes .waypoints file)
  Subscribes: /kml/field_boundary     (reads KML directly)

disease_detection_node.py        →   disease_detection_service.py
  Subscribes: /camera/image_raw       (reads cv2.VideoCapture)
  Subscribes: /mavros/global_pos      (reads MAVLink directly)
  Publishes:  /detection/hotspots     (writes CSV)
```

---

## Developer Docs

See the [`develop_debug/`](./develop_debug/) folder for in-depth explanations of every library used, with annotated code snippets and the reasoning behind each design decision:

| File | Covers |
|---|---|
| [`01_opencv.md`](./develop_debug/01_opencv.md) | How OpenCV is used for HSV masking, morphology, contour detection, and the debug GUI |
| [`02_numpy.md`](./develop_debug/02_numpy.md) | NumPy array operations behind mask logic and coordinate math |
| [`03_pymavlink.md`](./develop_debug/03_pymavlink.md) | MAVLink protocol, heartbeat handshake, GPS message parsing |
| [`04_threading.md`](./develop_debug/04_threading.md) | Why GPS runs on a daemon thread and how shared state is kept safe |
| [`05_coordinate_math.md`](./develop_debug/05_coordinate_math.md) | WGS-84, ENU projections, haversine, and pixel-to-GPS math |
| [`06_stdlib_kml.md`](./develop_debug/06_stdlib_kml.md) | Zero-dependency KML parsing with `xml.etree.ElementTree` |
| [`07_csv_logging.md`](./develop_debug/07_csv_logging.md) | Append-safe CSV logging pattern and output schema |

---

## File Map

```
Drone1-Standalone/
├── waypoint_generator.py              # Script 1 — KML → .waypoints
├── disease_detection_service.py       # Script 2 — Camera + GPS → disease CSV
├── README.md                          # This file
├── missions/                          # ← DROP YOUR KML FILES HERE
│   ├── README.md                      #   instructions
│   ├── your_field.kml                 #   (you add this)
│   └── your_field_waypoints_*.waypoints  # (auto-generated output)
├── develop_debug/                     # Library docs with code snippets
│   ├── 01_opencv.md
│   ├── 02_numpy.md
│   ├── 03_pymavlink.md
│   ├── 04_threading.md
│   ├── 05_coordinate_math.md
│   ├── 06_stdlib_kml.md
│   └── 07_csv_logging.md
└── disease_log_YYYYMMDD.csv           # (auto-generated by detection service)
```

---

## Requirements

```
Python >= 3.9

# For waypoint_generator.py:
  (none — uses stdlib only)

# For disease_detection_service.py:
pip install opencv-python numpy pymavlink
```

> **Tested on:** Windows 11 (COM port), Ubuntu 22.04 (ttyUSB0 / UDP)
