# 00 — System Architecture

## Overview

The drone disease detection system consists of two main Python scripts:

| Script | Purpose | Dependencies |
|--------|---------|--------------|
| `disease_detection_service.py` | Real-time detection + GPS logging | opencv-python, numpy, pymavlink |
| `waypoint_generator.py` | KML → ArduPilot waypoints (offline) | stdlib only |

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ARKAIRO System Architecture                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  disease_detection_service.py                                             │
│  ─────────────────────────────────                                          │
│                                                                             │
│  ┌──────────────┐    ┌─────────────────┐    ┌──────────────────────────┐  │
│  │  OpenCV     │───▶│ DiseaseDetector │───▶│ CSV Logger               │  │
│  │  Camera     │    │ - HSV masking   │    │ disease_geotag_<date>.csv│ │
│  └──────────────┘    │ - Contour filter│    └──────────────────────────┘  │
│         │             │ - Geotagging   │                                  │
│         ▼             └─────────────────┘                                  │
│  ┌──────────────┐                                                         │
│  │  GpsReader   │◀───────────────── MAVLink ◀────────────────┐             │
│  │  (thread)    │     (serial / UDP)                      │             │
│  └──────────────┘                                          │             │
│        │            ┌─────────────────┐                    │             │
│        └───────────▶│ pixel_to_gps    │                    │             │
│                     │ (nadir proj.)   │◀───────────────────┘             │
│                     └─────────────────┘                                  │
│                                                                             │
│  config.ini (modes: dev / pi5 / ...)                                      │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  waypoint_generator.py                                                    │
│  ────────────────────────                                                  │
│                                                                             │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐  │
│  │ KML      │───▶│ buffer       │───▶│ lawnmower   │───▶│ .waypoints  │  │
│  │ parse    │    │ polygon      │    │ scan lines  │    │ export      │  │
│  └──────────┘    └──────────────┘    └─────────────┘    └────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

### 1. Camera → Detection

```
VideoCapture (USB/RTSP/Video)
       │
       ▼ frame (H, W, 3) BGR
       │
       ▼ cv2.cvtColor(..., BGR2HSV)
       │
       ▼ cv2.inRange(yellow_hsv)  ──▶ yellow_mask
       ▼ cv2.inRange(green_hsv)   ──▶ green_mask
       ▼ cv2.inRange(brown_hsv)   ──▶ brown_mask (excluded)
       │
       ▼ morphological cleanup (OPEN/CLOSE)
       │
       ▼ vegetation context: yellow AND dilated(green)
       │
       ▼ cv2.findContours → filter by area/shape
       │
       ▼ for each detection:
          - pixel_to_gps() → lat/lon
          - check dupe: haversine() vs recent
          - log to CSV
```

### 2. GPS Sources (priority order)

```
1. MAVLink GLOBAL_POSITION_INT (EKF-fused) ── preferred
2. MAVLink GPS_RAW_INT (raw GPS, if fix >= 3D)
3. ArduPilot console "gps status" command (via SERIAL_CONTROL)
4. Config.GPS_FALLBACK_* (hardcoded)
```

The `GpsReader` thread continuously updates `GpsState` which the main thread reads lock-free.

---

## Config System

`config.ini` provides named modes that override defaults:

```ini
[dev]
serial_port = COM3
baud = 57600
camera_source = 0
show_gui = true

[pi5]
serial_port = /dev/ttyUSB0
baud = 115200
camera_source = rtsp://192.168.1.10:8554/stream
show_gui = false
```

CLI args `--mode`, `--port`, `--baud`, `--cam`, `--no-gui` override both.

---

## Output Files

| File | Location | Format |
|------|----------|--------|
| Detection log | `missions/disease_geotag_<timestamp>.csv` | CSV |
| Waypoints | `missions/<name>_waypoints_<timestamp>.waypoints` | ArduPilot QGC WPL 110 |

---

## Dependencies Summary

```bash
# Core (detection service)
pip install opencv-python numpy pymavlink

# Waypoint generator: stdlib only (no pip install needed)
```