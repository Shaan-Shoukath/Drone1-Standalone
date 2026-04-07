# 05 — Coordinate Math (stdlib `math`)

## Overview

Both scripts implement non-trivial geospatial math using **only Python's built-in `math` module** (no scipy, no shapely). This doc explains every formula used and the physical reasoning behind it.

---

## 1. WGS-84 Earth Model

All coordinate math uses the WGS-84 ellipsoid — the same reference model used by GPS receivers and Google Maps.

```python
def _wgs84_constants():
    a  = 6378137.0          # semi-major axis (equatorial radius), metres
    f  = 1.0 / 298.257223563  # flattening
    e2 = f * (2 - f)        # first eccentricity squared
    return a, f, e2
```

Earth is not a perfect sphere — it bulges at the equator. `a` is the equatorial radius (6,378 km), and `f` describes how much it's squashed at the poles.

---

## 2. LatLon → ECEF (Earth-Centred Earth-Fixed)

```python
def latlon_to_ecef(lat, lon, alt=0.0):
    a, _, e2 = _wgs84_constants()
    lr, lo = math.radians(lat), math.radians(lon)

    # N = radius of curvature in the prime vertical
    N = a / math.sqrt(1 - e2 * math.sin(lr) ** 2)

    x = (N + alt) * math.cos(lr) * math.cos(lo)
    y = (N + alt) * math.cos(lr) * math.sin(lo)
    z = (N * (1 - e2) + alt) * math.sin(lr)
    return x, y, z
```

ECEF is a 3D Cartesian system centred at Earth's centre of mass. X points toward the Prime Meridian, Z toward the North Pole. Converting to ECEF is necessary as an intermediate step for ENU projection.

---

## 3. ECEF → LatLon (Iterative Bowring Method)

```python
def ecef_to_latlon(x, y, z):
    a, _, e2 = _wgs84_constants()
    lon = math.degrees(math.atan2(y, x))         # straightforward
    p   = math.sqrt(x * x + y * y)               # distance from Z-axis
    lat = math.atan2(z, p * (1 - e2))            # initial estimate

    for _ in range(5):                            # 5 iterations → cm accuracy
        N   = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - N
        lat = math.atan2(z, p * (1 - e2 * (N / (N + alt))))

    return math.degrees(lat), lon, alt
```

Inverting the ECEF transform has no closed-form solution for an ellipsoid. 5 Newton-Raphson iterations converge to centimetre accuracy — fast enough for real-time use.

---

## 4. LatLon → ENU (Local Tangent Plane)

ENU (East-North-Up) is a local coordinate system centred on a reference point (e.g., the polygon centroid). "East" is +X, "North" is +Y, "Up" is +Z — all in metres.

```python
def latlon_to_enu(lat, lon, lat0, lon0, alt=0.0, alt0=0.0):
    # Step 1: Both points to ECEF
    x,  y,  z  = latlon_to_ecef(lat,  lon,  alt)
    x0, y0, z0 = latlon_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0    # difference vector

    # Step 2: Rotate ECEF difference into local ENU frame
    lr0, lo0 = math.radians(lat0), math.radians(lon0)
    sl, cl   = math.sin(lr0), math.cos(lr0)
    slo, clo = math.sin(lo0), math.cos(lo0)

    e =  -slo*dx  + clo*dy
    n =  -sl*clo*dx - sl*slo*dy + cl*dz
    u =   cl*clo*dx + cl*slo*dy + sl*dz
    return e, n, u
```

**Why ENU for the waypoint generator?**  
Working in metres (ENU) is much easier than working in degrees (lat/lon) when you need to:
- Offset polygon edges by a fixed number of metres (buffer)
- Space parallel scan lines at a fixed metre interval
- Compute intersections with geometry

---

## 5. Haversine Distance

```python
def haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6371000   # mean Earth radius in metres
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))
```

The **haversine formula** computes great-circle distance between two points on a sphere. Error vs. WGS-84 ellipsoid is < 0.5% for distances < 1000 km — more than sufficient for our 50–200 m field sizes.

**Uses in this project:**
- `GPS_DEDUP_M` check: is the new detection within 3 m of an already-logged hotspot?
- Waypoint ordering: which end of a scan line is closest to home?

---

## 6. Pixel → GPS Ground Coordinate (Nadir Pinhole Projection)

This converts a pixel location in the camera frame to a GPS coordinate on the ground, assuming the camera points straight down (nadir).

```python
def pixel_to_gps(px, py, frame_w, frame_h, gps):
    hfov = math.radians(Config.HFOV_DEG)  # horizontal field of view
    vfov = math.radians(Config.VFOV_DEG)  # vertical field of view

    alt = max(gps.alt, 0.5)               # avoid division by zero
    gnd_w = 2 * alt * math.tan(hfov / 2)  # ground footprint width in metres
    gnd_h = 2 * alt * math.tan(vfov / 2)

    # Pixel offset from image centre, normalised to -0.5 … +0.5
    dx_pct = (px - frame_w / 2) / frame_w
    dy_pct = (py - frame_h / 2) / frame_h

    # Convert to metres East/North
    x_east  =  dx_pct * gnd_w
    y_north = -dy_pct * gnd_h   # image Y increases downward, geographic N upward

    # Offset from drone GPS position using flat-Earth approximation
    R_earth = 6378137.0
    dlat = (y_north / R_earth) * (180 / math.pi)
    dlon = (x_east  / (R_earth * math.cos(math.radians(gps.lat)))) * (180 / math.pi)

    return gps.lat + dlat, gps.lon + dlon
```

**Key insight — the `cos(lat)` correction:**  
One degree of longitude spans a smaller distance as you move toward the poles (since lines of longitude converge). The correction `/ cos(lat)` accounts for this — without it, eastern offsets would be over-estimated.

**Assumptions / limitations:**
- Camera is pointing straight down (nadir) with no roll/pitch
- Flat earth approximation is valid for small offsets (< 1 km) — error < 1 mm at our field scale
- FOV values in `Config` must match your actual lens

---

## 7. Polygon Inward Buffer

See [`waypoint_generator.py` → `create_buffer_polygon`]:

1. Convert polygon vertices to ENU (metres)
2. For each edge, compute the inward normal
3. Offset each edge inward by `BUFFER_M` metres
4. Find intersection of adjacent offset edges → new vertex

This shrinks the polygon by a fixed distance from every side simultaneously, preventing waypoints from flying too close to the field boundary.

---

## stdlib — No Install Needed

Everything in this file uses only `math` from the Python standard library.
