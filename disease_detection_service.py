#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║        ARKAIRO — Plant Disease Detection & Geotagging Service               ║
╚══════════════════════════════════════════════════════════════════════════════╝

Background service that:
  • Reads your camera continuously (USB cam, RTSP stream, or video file)
  • Pulls real GPS from drone via MAVLink (same telemetry port Mission Planner uses)
  • Detects yellow plant disease using HSV + vegetation context validation
  • Geotags every new detection with the drone's live GPS position
  • Logs to CSV  →  disease_log_<date>.csv
  • Shows a live OpenCV debug window (optional, close anytime)
  • Runs until you press  Ctrl+C

WORKFLOW:
  1. Connect drone telemetry to laptop (USB / radio — same as Mission Planner)
  2. Note the COM port or UDP address Mission Planner uses
  3. Edit SERIAL_PORT / BAUD below (or pass --port COM3 --baud 57600)
  4. python disease_detection_service.py
  5. Press  Ctrl+C  to stop

Dependencies:
    pip install opencv-python numpy pymavlink

GPS SOURCE (choose ONE — comment out the others in Config):
  A) MAVLink over serial    →  SERIAL_PORT = 'COM3',  BAUD = 57600
  B) MAVLink over UDP       →  SERIAL_PORT = 'udp:0.0.0.0:14551'
  C) Hardcoded fallback     →  GPS_FALLBACK at bottom of Config
"""

import os
import sys
import csv
import math
import time
import threading
import argparse
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple, List

import cv2
import numpy as np

# ── MAVLink import (graceful if not installed) ────────────────────────────────
try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False
    print("[WARN]  pymavlink not found — GPS will use fallback coordinates.")
    print("        Install with:  pip install pymavlink")


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  ← edit these for your setup
# ══════════════════════════════════════════════════════════════════════════════
class Config:
    # ── Camera ────────────────────────────────────────────────────────────────
    # Integer 0/1/2 = USB webcam index
    # String  = video file path or RTSP URL
    CAMERA_SOURCE    = 0

    # Processing framerate (skip frames to stay within this budget)
    TARGET_FPS       = 5.0

    # Show live OpenCV debug window (set False for headless / SSH)
    SHOW_GUI         = True

    # ── MAVLink / GPS ─────────────────────────────────────────────────────────
    # Serial port Mission Planner uses for telemetry.
    # Examples:  'COM3'   '/dev/ttyUSB0'   'udp:0.0.0.0:14551'
    SERIAL_PORT      = 'COM3'
    BAUD             = 57600

    # Fallback GPS used when no MAVLink fix is available
    # (set to your field's rough centre so CSV coordinates make sense)
    GPS_FALLBACK_LAT = 10.04793794706056
    GPS_FALLBACK_LON = 76.33000537139496
    GPS_FALLBACK_ALT = 6.7              # metres

    # ── Camera FOV (for GPS projection — edit to match your lens) ─────────────
    HFOV_DEG         = 60.0
    VFOV_DEG         = 45.0

    # ── HSV thresholds ────────────────────────────────────────────────────────
    YELLOW_HSV_MIN   = (15,  80,  80)
    YELLOW_HSV_MAX   = (40, 255, 255)
    GREEN_HSV_MIN    = (35,  30,  30)
    GREEN_HSV_MAX    = (85, 255, 255)
    BROWN_HSV_MIN    = ( 5,  10,  40)   # sand / soil — excluded
    BROWN_HSV_MAX    = (18, 100, 180)

    # ── Detection thresholds ─────────────────────────────────────────────────
    MIN_AREA_PX      = 250
    MIN_GREEN_NEARBY = 150
    GPS_DEDUP_M      = 3.0              # suppress re-logging same hotspot

    # ── Output ────────────────────────────────────────────────────────────────
    _SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
    OUTPUT_DIR       = _SCRIPT_DIR     # CSVs saved alongside this script


# ══════════════════════════════════════════════════════════════════════════════
# DATA STRUCTURES
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class GpsState:
    lat:    float = Config.GPS_FALLBACK_LAT
    lon:    float = Config.GPS_FALLBACK_LON
    alt:    float = Config.GPS_FALLBACK_ALT
    fix:    bool  = False               # True = real MAVLink fix


@dataclass
class Detection:
    latitude:       float
    longitude:      float
    pixel_x:        int
    pixel_y:        int
    area:           int
    severity:       str
    severity_score: int
    confidence:     float


# ══════════════════════════════════════════════════════════════════════════════
# GPS / MAVLink THREAD
# ══════════════════════════════════════════════════════════════════════════════

class GpsReader(threading.Thread):
    """
    Background thread that connects to the drone via MAVLink and
    continuously updates a shared GpsState object.

    If the connection fails or MAVLink is not installed the thread
    silently uses the fallback coordinates from Config.
    """

    def __init__(self, state: GpsState, port: str, baud: int):
        super().__init__(daemon=True)
        self.state   = state
        self.port    = port
        self.baud    = baud
        self._stop   = threading.Event()
        self.connected = False

    def stop(self):
        self._stop.set()

    def run(self):
        if not HAS_MAVLINK:
            return

        print(f"[GPS]   Connecting to {self.port} @ {self.baud} baud ...")
        try:
            conn = mavutil.mavlink_connection(self.port, baud=self.baud)
            conn.wait_heartbeat(timeout=10)
            print(f"[GPS]   Heartbeat received from system "
                  f"{conn.target_system} / component {conn.target_component}")
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)   # 5 Hz
            self.connected = True
        except Exception as e:
            print(f"[GPS]   Connection failed: {e}")
            print(f"[GPS]   Using fallback GPS  "
                  f"({self.state.lat:.6f}, {self.state.lon:.6f})")
            return

        while not self._stop.is_set():
            try:
                msg = conn.recv_match(
                    type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
                    blocking=True, timeout=2.0
                )
                if msg is None:
                    continue
                t = msg.get_type()
                if t == 'GLOBAL_POSITION_INT':
                    self.state.lat = msg.lat  / 1e7
                    self.state.lon = msg.lon  / 1e7
                    self.state.alt = msg.alt  / 1000.0   # mm → m
                    self.state.fix = True
                elif t == 'GPS_RAW_INT' and msg.fix_type >= 3:
                    self.state.lat = msg.lat  / 1e7
                    self.state.lon = msg.lon  / 1e7
                    self.state.alt = msg.alt  / 1000.0
                    self.state.fix = True
            except Exception:
                time.sleep(0.1)


# ══════════════════════════════════════════════════════════════════════════════
# GPS HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6371000
    a = (math.sin(math.radians(lat2 - lat1) / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


def pixel_to_gps(px: int, py: int, frame_w: int, frame_h: int,
                 gps: GpsState) -> Tuple[float, float]:
    """
    Nadir-camera pinhole projection: pixel → GPS ground coordinate.
    Offset from image centre × ground footprint size at drone altitude.
    """
    hfov = math.radians(Config.HFOV_DEG)
    vfov = math.radians(Config.VFOV_DEG)

    alt = max(gps.alt, 0.5)                     # avoid division by zero
    gnd_w = 2 * alt * math.tan(hfov / 2)        # ground width in metres
    gnd_h = 2 * alt * math.tan(vfov / 2)

    dx_pct = (px - frame_w / 2) / frame_w       # -0.5 … 0.5
    dy_pct = (py - frame_h / 2) / frame_h

    x_east  =  dx_pct * gnd_w
    y_north = -dy_pct * gnd_h                   # image Y is flipped vs north

    R_earth = 6378137.0
    dlat = (y_north / R_earth) * (180.0 / math.pi)
    dlon = (x_east  / (R_earth * math.cos(math.radians(gps.lat)))) * (180.0 / math.pi)

    return gps.lat + dlat, gps.lon + dlon


# ══════════════════════════════════════════════════════════════════════════════
# DETECTION ENGINE
# ══════════════════════════════════════════════════════════════════════════════

class DiseaseDetector:
    def __init__(self):
        self.yellow_min = np.array(Config.YELLOW_HSV_MIN)
        self.yellow_max = np.array(Config.YELLOW_HSV_MAX)
        self.green_min  = np.array(Config.GREEN_HSV_MIN)
        self.green_max  = np.array(Config.GREEN_HSV_MAX)
        self.brown_min  = np.array(Config.BROWN_HSV_MIN)
        self.brown_max  = np.array(Config.BROWN_HSV_MAX)

        self.kernel_s   = np.ones((3, 3), np.uint8)
        self.kernel_l   = np.ones((5, 5), np.uint8)

        self.logged_locs: List[Tuple[float, float]] = []
        self.total_detections = 0

    def _is_dup(self, lat: float, lon: float) -> bool:
        return any(haversine(lat, lon, p[0], p[1]) < Config.GPS_DEDUP_M
                   for p in self.logged_locs)

    def detect(self, frame: np.ndarray,
               gps: GpsState) -> Tuple[List[Detection], np.ndarray, np.ndarray]:
        """
        Run disease detection on one frame.
        Returns: (detections, yellow_mask, green_mask)
        """
        h, w = frame.shape[:2]
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        yel  = cv2.inRange(hsv, self.yellow_min, self.yellow_max)
        grn  = cv2.inRange(hsv, self.green_min,  self.green_max)
        brn  = cv2.inRange(hsv, self.brown_min,  self.brown_max)

        # Morphological cleanup
        yel = cv2.morphologyEx(yel, cv2.MORPH_OPEN,  self.kernel_s)
        yel = cv2.morphologyEx(yel, cv2.MORPH_CLOSE, self.kernel_l)
        grn = cv2.morphologyEx(grn, cv2.MORPH_OPEN,  self.kernel_s)
        grn = cv2.morphologyEx(grn, cv2.MORPH_CLOSE, self.kernel_l)

        # Sand exclusion
        yel = cv2.bitwise_and(yel, cv2.bitwise_not(brn))

        # Only detect yellow when embedded in green vegetation
        green_total = int(np.sum(grn > 0))
        if green_total > 1000:
            grn_dilated = cv2.dilate(grn, self.kernel_l, iterations=5)
            yel = cv2.bitwise_and(yel, grn_dilated)

        contours, _ = cv2.findContours(yel, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < Config.MIN_AREA_PX:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            cx, cy = bx + bw // 2, by + bh // 2

            # Validation 1: green nearby
            mg = 30
            y1, y2 = max(0, by - mg), min(h, by + bh + mg)
            x1, x2 = max(0, bx - mg), min(w, bx + bw + mg)
            grn_near = int(np.sum(grn[y1:y2, x1:x2] > 0))
            if green_total > 2000 and grn_near < Config.MIN_GREEN_NEARBY:
                continue

            # Validation 2: not dominated by sand
            brn_near = int(np.sum(brn[y1:y2, x1:x2] > 0))
            if brn_near > grn_near * 1.5:
                continue

            # Validation 3: shape
            aspect  = float(bw) / bh if bh > 0 else 0
            perim   = cv2.arcLength(cnt, True)
            compact = (4 * math.pi * area) / (perim ** 2) if perim > 0 else 0
            if not (0.25 <= aspect <= 4.0 and compact > 0.25):
                continue

            # Geotag
            lat, lon = pixel_to_gps(cx, cy, w, h, gps)

            # Deduplication
            if self._is_dup(lat, lon):
                continue

            self.logged_locs.append((lat, lon))
            if len(self.logged_locs) > 500:
                self.logged_locs.pop(0)

            # Severity
            if area >= 1000:
                severity, score = 'SEVERE',   3
            elif area >= 500:
                severity, score = 'MODERATE', 2
            else:
                severity, score = 'MILD',     1

            confidence = min(1.0, area / (Config.MIN_AREA_PX * 10))
            self.total_detections += 1

            detections.append(Detection(
                latitude=lat, longitude=lon,
                pixel_x=cx,  pixel_y=cy,
                area=int(area), severity=severity,
                severity_score=score, confidence=confidence,
            ))

        return detections, yel, grn

    # ── Debug visualisation ──────────────────────────────────────────────────
    def draw_debug(self, frame: np.ndarray, detections: List[Detection],
                   gps: GpsState, fps: float) -> np.ndarray:
        """2×2 debug grid: annotated feed | yellow mask | green mask | overlay."""
        h, w = frame.shape[:2]
        s = 0.5

        # Re-compute masks for visualisation (cheap — just for display)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yel = cv2.inRange(hsv, self.yellow_min, self.yellow_max)
        grn = cv2.inRange(hsv, self.green_min,  self.green_max)
        yel = cv2.morphologyEx(yel, cv2.MORPH_OPEN,  self.kernel_s)
        yel = cv2.morphologyEx(yel, cv2.MORPH_CLOSE, self.kernel_l)
        grn = cv2.morphologyEx(grn, cv2.MORPH_OPEN,  self.kernel_s)

        severity_col = {'MILD': (0, 200, 255), 'MODERATE': (0, 100, 255), 'SEVERE': (0, 0, 255)}

        # ── Panel 1: annotated camera ─────────────────────────────────────────
        ann = frame.copy()
        for det in detections:
            col = severity_col.get(det.severity, (255, 255, 255))
            cv2.circle(ann, (det.pixel_x, det.pixel_y), 18, col, 3)
            cv2.line(ann, (det.pixel_x - 22, det.pixel_y),
                          (det.pixel_x + 22, det.pixel_y), col, 2)
            cv2.line(ann, (det.pixel_x, det.pixel_y - 22),
                          (det.pixel_x, det.pixel_y + 22), col, 2)
            cv2.putText(ann, f"{det.severity} ({det.area}px)",
                        (det.pixel_x + 24, det.pixel_y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)
            cv2.putText(ann, f"{det.latitude:.5f},{det.longitude:.5f}",
                        (det.pixel_x + 24, det.pixel_y + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.37, (200, 200, 200), 1)

        # Stats overlay
        gps_color = (0, 255, 0) if gps.fix else (0, 140, 255)
        gps_label = "GPS: LIVE" if gps.fix else "GPS: FALLBACK"
        cv2.rectangle(ann, (0, 0), (390, 115), (25, 25, 25), -1)
        cv2.putText(ann, gps_label,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.60, gps_color, 2)
        cv2.putText(ann, f"Pos: {gps.lat:.5f}, {gps.lon:.5f}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)
        cv2.putText(ann, f"Alt: {gps.alt:.1f} m   FPS: {fps:.1f}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)
        cv2.putText(ann, f"Total detections: {self.total_detections}",
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)

        ann_s = cv2.resize(ann, None, fx=s, fy=s)

        # ── Panel 2: yellow mask ──────────────────────────────────────────────
        yel_bgr = cv2.cvtColor(yel, cv2.COLOR_GRAY2BGR)
        yel_bgr[yel > 0] = [0, 220, 220]
        yel_s = cv2.resize(yel_bgr, None, fx=s, fy=s)
        cv2.putText(yel_s, "Yellow Mask", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1)

        # ── Panel 3: green mask ───────────────────────────────────────────────
        grn_bgr = cv2.cvtColor(grn, cv2.COLOR_GRAY2BGR)
        grn_bgr[grn > 0] = [0, 200, 0]
        grn_s = cv2.resize(grn_bgr, None, fx=s, fy=s)
        cv2.putText(grn_s, "Green Mask (Plant Context)", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

        # ── Panel 4: yellow overlay on raw ────────────────────────────────────
        overlay = frame.copy()
        overlay[yel > 0] = [0, 220, 220]
        ov_s = cv2.resize(overlay, None, fx=s, fy=s)
        cv2.putText(ov_s, "Yellow Overlay", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1)

        top    = np.hstack([ann_s, yel_s])
        bottom = np.hstack([grn_s, ov_s])
        return np.vstack([top, bottom])


# ══════════════════════════════════════════════════════════════════════════════
# CSV LOGGING
# ══════════════════════════════════════════════════════════════════════════════

def init_csv(path: str):
    if not os.path.isfile(path):
        with open(path, 'w', newline='') as f:
            csv.writer(f).writerow([
                'Timestamp', 'Detection_ID', 'Latitude', 'Longitude',
                'Altitude_m', 'GPS_Source', 'Severity', 'Severity_Score',
                'Area_px', 'Pixel_X', 'Pixel_Y', 'Status',
            ])


def log_csv(path: str, det: Detection, det_id: int, gps: GpsState):
    with open(path, 'a', newline='') as f:
        csv.writer(f).writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f'DET{det_id:05d}',
            f'{det.latitude:.8f}',
            f'{det.longitude:.8f}',
            f'{gps.alt:.2f}',
            'LIVE' if gps.fix else 'FALLBACK',
            det.severity,
            det.severity_score,
            det.area,
            det.pixel_x,
            det.pixel_y,
            'DETECTED',
        ])


# ══════════════════════════════════════════════════════════════════════════════
# MAIN SERVICE LOOP
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Arkairo Plant Disease Detection Service')
    parser.add_argument('--port',   default=Config.SERIAL_PORT,
                        help='MAVLink port (e.g. COM3, /dev/ttyUSB0, udp:0.0.0.0:14551)')
    parser.add_argument('--baud',   type=int, default=Config.BAUD,
                        help='Serial baud rate (default 57600)')
    parser.add_argument('--cam',    default=None,
                        help='Camera source: 0/1 for USB cam, path or RTSP URL')
    parser.add_argument('--no-gui', action='store_true',
                        help='Disable OpenCV window (headless mode)')
    args = parser.parse_args()

    cam_source = int(args.cam) if (args.cam and args.cam.isdigit()) else \
                 (args.cam if args.cam else Config.CAMERA_SOURCE)
    show_gui   = Config.SHOW_GUI and not args.no_gui

    print()
    print("═" * 64)
    print("  ARKAIRO — Disease Detection & Geotagging Service")
    print("═" * 64)
    print(f"  Camera  : {cam_source}")
    print(f"  MAVLink : {args.port} @ {args.baud} baud")
    print(f"  Output  : {Config.OUTPUT_DIR}")
    print(f"  GUI     : {'on' if show_gui else 'off  (headless)'}")
    print("  Press   : Ctrl+C to stop")
    print("═" * 64)
    print()

    # ── GPS reader thread ─────────────────────────────────────────────────────
    gps = GpsState()
    gps_thread = GpsReader(gps, args.port, args.baud)
    gps_thread.start()

    # ── Camera ────────────────────────────────────────────────────────────────
    print(f"[CAM]   Opening camera: {cam_source}")
    cap = cv2.VideoCapture(cam_source)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera: {cam_source}")
        gps_thread.stop()
        sys.exit(1)
    print(f"[CAM]   Camera opened  "
          f"({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})")

    # ── CSV ───────────────────────────────────────────────────────────────────
    today    = datetime.now().strftime('%Y%m%d')
    csv_path = os.path.join(Config.OUTPUT_DIR, f"disease_log_{today}.csv")
    init_csv(csv_path)
    print(f"[LOG]   CSV log: {csv_path}")
    print()

    # ── Detection engine ──────────────────────────────────────────────────────
    detector = DiseaseDetector()

    min_interval = 1.0 / Config.TARGET_FPS
    last_process = 0.0
    fps_ema      = 0.0

    print("[INFO]  Service running — Ctrl+C to stop\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN]  Camera read failed — retrying ...")
                time.sleep(0.5)
                continue

            now = time.perf_counter()
            if now - last_process < min_interval:
                time.sleep(0.005)
                continue

            dt          = now - last_process if last_process > 0 else min_interval
            fps_ema     = 0.9 * fps_ema + 0.1 * (1.0 / dt) if fps_ema > 0 else 1.0 / dt
            last_process = now

            # ── Detection ─────────────────────────────────────────────────────
            detections, _, _ = detector.detect(frame, gps)

            # ── Log detected blobs ────────────────────────────────────────────
            for det in detections:
                log_csv(csv_path, det, detector.total_detections, gps)
                src = "LIVE" if gps.fix else "fallback"
                print(f"  [DET #{detector.total_detections:04d}]  "
                      f"{det.severity:8s}  "
                      f"lat={det.latitude:.6f}  lon={det.longitude:.6f}  "
                      f"area={det.area}px  GPS={src}")

            # ── GUI ───────────────────────────────────────────────────────────
            if show_gui:
                grid = detector.draw_debug(frame, detections, gps, fps_ema)
                cv2.imshow('Arkairo Disease Detection  [Ctrl+C to stop]', grid)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n[INFO]  Q pressed — stopping.")
                    break

    except KeyboardInterrupt:
        print(f"\n\n[INFO]  Stopped by user.")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print(f"\n{'═'*64}")
    print(f"  Session summary")
    print(f"    Total detections  : {detector.total_detections}")
    print(f"    CSV log           : {csv_path}")
    print(f"{'═'*64}\n")

    cap.release()
    if show_gui:
        cv2.destroyAllWindows()
    gps_thread.stop()


if __name__ == '__main__':
    main()
