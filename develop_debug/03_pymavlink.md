# 03 — pymavlink

## Why pymavlink?

MAVLink (Micro Air Vehicle Link) is the lightweight binary messaging protocol used by ArduPilot, PX4, Mission Planner, and QGroundControl to communicate with drones. `pymavlink` gives Python first-class access to this protocol — parsing incoming telemetry packets and sending commands without needing a full MAVROS/ROS2 stack.

In this project it has one job: **stream live GPS from the drone to the detection service**.

---

## 1. Graceful Import (Optional Dependency Pattern)

```python
try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False
    print("[WARN]  pymavlink not found — GPS will use fallback coordinates.")
```

**Why a try/except?**  
The detection service should still work for people who just want to test the vision pipeline on a laptop without a drone. If `pymavlink` is absent, `HAS_MAVLINK = False` and the thread returns immediately, leaving fallback GPS in place.

---

## 2. Creating a Connection

```python
from pymavlink import mavutil

# Serial port (USB telemetry radio / USB-to-serial):
conn = mavutil.mavlink_connection('COM3', baud=57600)

# UDP (Mission Planner forwarding / SITL simulator):
conn = mavutil.mavlink_connection('udp:0.0.0.0:14551')

# TCP:
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
```

`mavlink_connection` is smart enough to detect whether the string is a serial port, UDP, or TCP — so command-line users only need to change the `--port` argument.

---

## 3. Heartbeat Handshake

```python
conn.wait_heartbeat(timeout=10)
# Blocks until the drone (or SITL) sends its first HEARTBEAT message.
# Raises TimeoutError after 10 seconds if nothing arrives.

print(f"System {conn.target_system} / Component {conn.target_component}")
```

ArduPilot sends a HEARTBEAT at 1 Hz. Waiting for it:
1. Confirms the physical link is live
2. Populates `conn.target_system` / `target_component` needed to address further messages

---

## 4. Requesting GPS Data Stream

```python
conn.mav.request_data_stream_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,   # stream ID = GPS
    5,    # rate in Hz
    1,    # 1 = enable, 0 = disable
)
```

By default ArduPilot may not stream GPS at any useful rate. This call asks the autopilot to send `GLOBAL_POSITION_INT` and `GPS_RAW_INT` at 5 Hz.

---

## 5. Receiving GPS Messages

```python
while True:
    msg = conn.recv_match(
        type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
        blocking=True,
        timeout=2.0
    )
    if msg is None:
        continue   # timeout — no message in 2 s

    t = msg.get_type()

    if t == 'GLOBAL_POSITION_INT':
        # Fused position (EKF output) — preferred when available
        lat = msg.lat  / 1e7   # stored as integer degrees × 1e7
        lon = msg.lon  / 1e7
        alt = msg.alt  / 1000  # millimetres → metres

    elif t == 'GPS_RAW_INT' and msg.fix_type >= 3:
        # Raw GPS fix — 3+ = 3D fix
        lat = msg.lat  / 1e7
        lon = msg.lon  / 1e7
        alt = msg.alt  / 1000
```

**Integer encoding:** MAVLink stores lat/lon as `int32` multiplied by 1×10⁷ to avoid floating-point transmission costs. Always divide by `1e7`.

**`GLOBAL_POSITION_INT` vs `GPS_RAW_INT`:**

| Message | Source | When to prefer |
|---|---|---|
| `GLOBAL_POSITION_INT` | EKF fused (GPS + IMU + baro) | Always — smoother, more accurate |
| `GPS_RAW_INT` | Raw GNSS receiver | Fallback if EKF not running |

---

## 6. Why a Background Thread?

```python
class GpsReader(threading.Thread):
    def __init__(self, state: GpsState, port, baud):
        super().__init__(daemon=True)   # dies when main thread exits
        self.state = state

    def run(self):
        # ... connects and loops forever updating self.state ...
```

MAVLink `recv_match` is **blocking** — it waits up to 2 seconds for each message. Putting it on the main thread would stall the camera loop at 0.5 fps. A daemon thread updates `GpsState` in the background; the camera loop reads the latest values without waiting.

See [`04_threading.md`](./04_threading.md) for the full thread safety discussion.

---

## 7. GPS Fix Types (`fix_type`)

| Value | Meaning |
|---|---|
| 0 | No GPS |
| 1 | No fix |
| 2 | 2D fix (altitude unreliable) |
| 3 | 3D fix ✓ |
| 4 | DGPS (differential) |
| 5 | RTK float |
| 6 | RTK fixed (sub-centimetre) |

We gate on `>= 3` to ensure altitude is valid for the pixel-to-GPS projection.

---

## Install

```bash
pip install pymavlink
```
