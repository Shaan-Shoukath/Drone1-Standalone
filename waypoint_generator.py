#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║             ARKAIRO — KML Survey Waypoint Generator                         ║
╚══════════════════════════════════════════════════════════════════════════════╝

Reads a KML polygon file (drawn in Google Earth / My Maps) and generates an
ArduPilot-compatible .waypoints mission file using a lawnmower survey pattern.

WORKFLOW:
  1. Draw your field boundary as a polygon in Google Earth / My Maps
  2. Export it as a KML file
  3. Run this script:
       python waypoint_generator.py path/to/field.kml
  4. The .waypoints file is saved to  drone1_ws/missions/
  5. Open Mission Planner → Flight Plan → Load WP → select the file
  6. Upload to the drone and fly

Dependencies: Python 3 standard library ONLY  (no pip install needed)

Usage:
    python waypoint_generator.py                    # embedded SOE demo polygon
    python waypoint_generator.py path/to/field.kml  # your own KML
"""

import os
import sys
import math
import xml.etree.ElementTree as ET
import re
from datetime import datetime
from typing import List, Tuple, Optional, Dict


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — tweak these for your demo
# ══════════════════════════════════════════════════════════════════════════════
class Config:
    # ── Flight parameters ──────────────────────────────────────────────────────
    ALTITUDE_M       = 6.7    # metres AGL (≈22 ft, our actual field altitude)
    LANE_SPACING_M   = 5.0    # metres between parallel scan lines
    BUFFER_M         = 2.0    # inward boundary buffer in metres

    # ── Home position  (SOE campus gate, Thrissur — update to your launch point) ──
    HOME_LAT         = 10.0478
    HOME_LON         = 76.3303

    # ── Output folder: save directly into the missions folder Mission Planner reads ──
    _SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
    OUTPUT_DIR       = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', 'drone1_ws', 'missions'))


# ══════════════════════════════════════════════════════════════════════════════
# DEMO KML — the SOE sports field polygon embedded so the script works offline
# ══════════════════════════════════════════════════════════════════════════════
DEMO_KML_CONTENT = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <Placemark>
    <name>SOE SF Demo</name>
    <Polygon>
      <outerBoundaryIs>
        <LinearRing>
          <coordinates>
            76.33000537139496,10.04793794706056,0
            76.33054440781922,10.04773133612099,0
            76.33057468610598,10.04804336058582,0
            76.33018802896846,10.04820540593877,0
            76.33000537139496,10.04793794706056,0
          </coordinates>
        </LinearRing>
      </outerBoundaryIs>
    </Polygon>
  </Placemark>
</Document>
</kml>"""


# ══════════════════════════════════════════════════════════════════════════════
# COORDINATE MATH HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _wgs84_constants():
    a  = 6378137.0
    f  = 1.0 / 298.257223563
    e2 = f * (2 - f)
    return a, f, e2


def latlon_to_ecef(lat: float, lon: float, alt: float = 0.0):
    a, _, e2 = _wgs84_constants()
    lr, lo = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(lr) ** 2)
    x = (N + alt) * math.cos(lr) * math.cos(lo)
    y = (N + alt) * math.cos(lr) * math.sin(lo)
    z = (N * (1 - e2) + alt) * math.sin(lr)
    return x, y, z


def ecef_to_latlon(x: float, y: float, z: float):
    a, _, e2 = _wgs84_constants()
    lon = math.degrees(math.atan2(y, x))
    p   = math.sqrt(x * x + y * y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(5):
        N   = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - N
        lat = math.atan2(z, p * (1 - e2 * (N / (N + alt))))
    N   = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), lon, alt


def latlon_to_enu(lat, lon, lat0, lon0, alt=0.0, alt0=0.0):
    x,  y,  z  = latlon_to_ecef(lat,  lon,  alt)
    x0, y0, z0 = latlon_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lr0, lo0 = math.radians(lat0), math.radians(lon0)
    sl, cl = math.sin(lr0), math.cos(lr0)
    slo, clo = math.sin(lo0), math.cos(lo0)
    e = -slo * dx + clo * dy
    n = -sl * clo * dx - sl * slo * dy + cl * dz
    u =  cl * clo * dx + cl * slo * dy + sl * dz
    return e, n, u


def enu_to_latlon(e, n, lat0, lon0, u=0.0, alt0=0.0):
    x0, y0, z0 = latlon_to_ecef(lat0, lon0, alt0)
    lr0, lo0 = math.radians(lat0), math.radians(lon0)
    sl, cl = math.sin(lr0), math.cos(lr0)
    slo, clo = math.sin(lo0), math.cos(lo0)
    dx = -slo * e - sl * clo * n + cl * clo * u
    dy =  clo * e - sl * slo * n + cl * slo * u
    dz =  cl  * n + sl * u
    lat, lon, _ = ecef_to_latlon(x0 + dx, y0 + dy, z0 + dz)
    return lat, lon


def haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Distance between two (lat, lon) points in metres."""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ══════════════════════════════════════════════════════════════════════════════
# KML PARSING
# ══════════════════════════════════════════════════════════════════════════════

def parse_kml(kml_source: str) -> List[Tuple[float, float]]:
    """
    Parse KML from a file path or raw XML string.
    Returns list of (lat, lon) tuples.
    """
    if os.path.isfile(kml_source):
        tree = ET.parse(kml_source)
        root = tree.getroot()
    else:
        root = ET.fromstring(kml_source)

    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    if root.tag.startswith('{'):
        m = re.match(r'\{([^}]+)\}', root.tag)
        if m:
            namespace = {'kml': m.group(1)}

    paths = [
        './/kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates',
        './/kml:coordinates',
    ]
    coords_elem = None
    for path in paths:
        coords_elem = root.find(path, namespace)
        if coords_elem is not None:
            break
    # Fallback without namespace
    if coords_elem is None:
        coords_elem = root.find('.//coordinates')

    if coords_elem is None:
        raise ValueError("No <coordinates> element found in KML")

    coordinates = []
    for token in coords_elem.text.strip().split():
        parts = token.strip().split(',')
        if len(parts) >= 2:
            coordinates.append((float(parts[1]), float(parts[0])))  # (lat, lon)

    if len(coordinates) < 3:
        raise ValueError("KML polygon needs at least 3 points")

    return coordinates


# ══════════════════════════════════════════════════════════════════════════════
# BUFFER POLYGON (inward shrink)
# ══════════════════════════════════════════════════════════════════════════════

def _line_intersect(p1, p2, p3, p4):
    x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-10:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def create_buffer_polygon(polygon: List[Tuple[float, float]],
                          buffer_m: float) -> List[Tuple[float, float]]:
    """Shrink polygon inward by buffer_m metres."""
    if len(polygon) < 3:
        return polygon

    clat = sum(p[0] for p in polygon) / len(polygon)
    clon = sum(p[1] for p in polygon) / len(polygon)

    enu = [latlon_to_enu(lat, lon, clat, clon)[:2] for lat, lon in polygon]
    n   = len(enu)

    offset_edges = []
    for i in range(n):
        p1, p2 = enu[i], enu[(i + 1) % n]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        nx, ny = -dy / length, dx / length          # inward normal (CCW)
        op1 = (p1[0] + nx * buffer_m, p1[1] + ny * buffer_m)
        op2 = (p2[0] + nx * buffer_m, p2[1] + ny * buffer_m)
        offset_edges.append((op1, op2))

    if len(offset_edges) < 3:
        return polygon

    buffered = []
    for i in range(len(offset_edges)):
        e1 = offset_edges[i]
        e2 = offset_edges[(i + 1) % len(offset_edges)]
        pt = _line_intersect(e1[0], e1[1], e2[0], e2[1])
        if pt:
            buffered.append(pt)
        else:
            mid = ((e1[1][0] + e2[0][0]) / 2, (e1[1][1] + e2[0][1]) / 2)
            buffered.append(mid)

    return [enu_to_latlon(e, n, clat, clon) for e, n in buffered]


# ══════════════════════════════════════════════════════════════════════════════
# SURVEY WAYPOINT GENERATION (KMLToWaypointV8 algorithm)
# ══════════════════════════════════════════════════════════════════════════════

def _seg_intersect(line_start, line_end, seg_start, seg_end):
    x1, y1 = line_start; x2, y2 = line_end
    x3, y3 = seg_start;  x4, y4 = seg_end
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-10:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / d
    if 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None


def polygon_line_intersections(line_start, line_end, polygon_enu):
    hits = []
    for i in range(len(polygon_enu)):
        pt = _seg_intersect(line_start, line_end,
                            polygon_enu[i], polygon_enu[(i + 1) % len(polygon_enu)])
        if pt:
            if not any(abs(pt[0] - h[0]) < 1e-8 and abs(pt[1] - h[1]) < 1e-8 for h in hits):
                hits.append(pt)
    return hits


def generate_waypoints(polygon: List[Tuple[float, float]],
                       spacing_m: float,
                       buffer_m: float,
                       home: Tuple[float, float]) -> List[Tuple[float, float]]:
    """
    Generate lawnmower survey waypoints for the given polygon.
    Returns ordered list of (lat, lon) flight points.
    """
    # 1. Buffer
    buf_poly = create_buffer_polygon(polygon, buffer_m)
    if len(buf_poly) < 3:
        buf_poly = polygon

    # 2. Convert to local ENU
    clat = sum(p[0] for p in buf_poly) / len(buf_poly)
    clon = sum(p[1] for p in buf_poly) / len(buf_poly)
    enu  = [latlon_to_enu(lat, lon, clat, clon)[:2] for lat, lon in buf_poly]

    # 3. Find longest side → determine scan direction
    max_len, best_angle = 0.0, 0.0
    for i in range(len(enu)):
        dx = enu[(i + 1) % len(enu)][0] - enu[i][0]
        dy = enu[(i + 1) % len(enu)][1] - enu[i][1]
        L  = math.hypot(dx, dy)
        if L > max_len:
            max_len = L
            best_angle = math.degrees(math.atan2(dy, dx)) % 360

    angle_rad = math.radians(best_angle)
    dir_u  = (math.cos(angle_rad), math.sin(angle_rad))     # along scan lines
    perp_u = (-dir_u[1], dir_u[0])                          # perpendicular

    # 4. Generate parallel lines
    es, ns = [p[0] for p in enu], [p[1] for p in enu]
    diag   = math.hypot(max(es) - min(es), max(ns) - min(ns))
    max_off = diag / 2 + spacing_m * 2

    all_lines = []
    k = 0
    while True:
        offset = k * spacing_m
        if offset > max_off:
            break
        for sign in ([-1, 1] if k != 0 else [1]):
            off = offset * sign
            p0  = (perp_u[0] * off, perp_u[1] * off)
            L   = diag * 2 + 1000
            s   = (p0[0] - dir_u[0] * L, p0[1] - dir_u[1] * L)
            e_  = (p0[0] + dir_u[0] * L, p0[1] + dir_u[1] * L)
            hits = polygon_line_intersections(s, e_, enu)
            if len(hits) >= 2:
                hits.sort(key=lambda p: (p[0] - p0[0]) * dir_u[0] + (p[1] - p0[1]) * dir_u[1])
                all_lines.append({
                    'offset': off,
                    'start':  enu_to_latlon(hits[0][0],  hits[0][1],  clat, clon),
                    'end':    enu_to_latlon(hits[-1][0], hits[-1][1], clat, clon),
                })
        k += 1

    all_lines.sort(key=lambda x: x['offset'])

    if not all_lines:
        return []

    # 5. Optimise snake ordering (start nearest to home)
    waypoints = []
    for i, line in enumerate(all_lines):
        if i == 0:
            # Pick end closest to home for start
            d_start = haversine(home, line['start'])
            d_end   = haversine(home, line['end'])
            if d_start <= d_end:
                waypoints += [line['start'], line['end']]
            else:
                waypoints += [line['end'], line['start']]
        else:
            prev = waypoints[-1]
            if haversine(prev, line['start']) <= haversine(prev, line['end']):
                waypoints += [line['start'], line['end']]
            else:
                waypoints += [line['end'], line['start']]

    # If flying the list backwards brings us closer to home overall, reverse it
    first_dist = haversine(waypoints[0],  home)
    last_dist  = haversine(waypoints[-1], home)
    if last_dist < first_dist * 0.8:
        waypoints = list(reversed(waypoints))

    return waypoints


# ══════════════════════════════════════════════════════════════════════════════
# ARDUPILOT .WAYPOINTS EXPORT
# ══════════════════════════════════════════════════════════════════════════════

def export_waypoints(waypoints: List[Tuple[float, float]],
                     home: Tuple[float, float],
                     altitude_m: float,
                     output_path: str):
    """Write ArduPilot QGC WPL 110 waypoints file."""
    with open(output_path, 'w') as f:
        f.write("QGC WPL 110\n")
        idx = 0

        # Item 0: Home point (CMD 16 at seq 0 = home)
        f.write(f"{idx}\t0\t0\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                f"{home[0]:.6f}\t{home[1]:.6f}\t0.100000\t1\n")
        idx += 1

        # Item 1: Takeoff (CMD 22)
        f.write(f"{idx}\t0\t3\t22\t0.000000\t0.000000\t0.000000\t0.000000\t"
                f"0.000000\t0.000000\t{altitude_m:.6f}\t1\n")
        idx += 1

        # Survey waypoints (CMD 16)
        for lat, lon in waypoints:
            f.write(f"{idx}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                    f"{lat:.6f}\t{lon:.6f}\t{altitude_m:.6f}\t1\n")
            idx += 1

        # RTL (CMD 20)
        f.write(f"{idx}\t0\t0\t20\t0.000000\t0.000000\t0.000000\t0.000000\t"
                f"0.000000\t0.000000\t0.000000\t1\n")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # ── Resolve KML source ────────────────────────────────────────────────────
    if len(sys.argv) > 1:
        kml_arg = sys.argv[1]
        if not os.path.isfile(kml_arg):
            print(f"[ERROR] KML file not found: {kml_arg}")
            sys.exit(1)
        kml_source  = kml_arg
        kml_basename = os.path.splitext(os.path.basename(kml_arg))[0]
        print(f"[INFO]  Using KML: {kml_arg}")
    else:
        print("[INFO]  No KML file provided — using embedded SOE sports field demo polygon")
        kml_source   = DEMO_KML_CONTENT
        kml_basename = "SOE_demo"

    # ── Parse ─────────────────────────────────────────────────────────────────
    print("[STEP 1] Parsing KML polygon...")
    polygon = parse_kml(kml_source)
    print(f"         → {len(polygon)} boundary vertices found")

    # ── Generate ──────────────────────────────────────────────────────────────
    home = (Config.HOME_LAT, Config.HOME_LON)
    print(f"[STEP 2] Generating survey waypoints ...")
    print(f"         Altitude    : {Config.ALTITUDE_M} m")
    print(f"         Lane spacing: {Config.LANE_SPACING_M} m")
    print(f"         Buffer      : {Config.BUFFER_M} m")
    print(f"         Home        : {home}")

    waypoints = generate_waypoints(polygon,
                                   spacing_m=Config.LANE_SPACING_M,
                                   buffer_m=Config.BUFFER_M,
                                   home=home)

    if not waypoints:
        print("[ERROR] No waypoints generated — polygon may be too small or buffer too large.")
        sys.exit(1)

    print(f"         → {len(waypoints)} survey waypoints generated")

    # ── Export ────────────────────────────────────────────────────────────────
    os.makedirs(Config.OUTPUT_DIR, exist_ok=True)
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"{kml_basename}_waypoints_{ts}.waypoints"
    out_path = os.path.join(Config.OUTPUT_DIR, filename)

    print(f"[STEP 3] Exporting to:")
    print(f"         {out_path}")
    export_waypoints(waypoints, home, Config.ALTITUDE_M, out_path)

    print(f"\n{'═'*62}")
    print(f"  ✓  Mission file saved:")
    print(f"     {out_path}")
    print(f"")
    print(f"     Items: Home + Takeoff + {len(waypoints)} survey WPs + RTL")
    print(f"     Total: {len(waypoints) + 3} mission items")
    print(f"{'═'*62}")
    print()
    print("  NEXT STEPS:")
    print("    1. Open Mission Planner")
    print("    2. Flight Plan tab → Load WP")
    print(f"    3. Navigate to:  {Config.OUTPUT_DIR}")
    print(f"    4. Select:  {filename}")
    print("    5. Connect drone → Upload")
    print()

    # ── Pretty-print first few waypoints ─────────────────────────────────────
    print("  First 6 survey waypoints:")
    for i, (lat, lon) in enumerate(waypoints[:6]):
        print(f"    WP{i+1:02d}  lat={lat:.6f}  lon={lon:.6f}")
    if len(waypoints) > 6:
        print(f"    ... ({len(waypoints) - 6} more)")
    print()


if __name__ == '__main__':
    main()
