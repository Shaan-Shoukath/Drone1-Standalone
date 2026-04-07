# 04 — Threading (`threading` — stdlib)

## Why Threading?

The detection service needs to do two independent things *at the same time*:

1. **Camera loop** — grab frame → detect → show GUI → repeat at ~5 fps
2. **GPS reader** — wait for MAVLink messages → update coordinates → repeat at ~5 Hz

If GPS ran on the main thread, `conn.recv_match(blocking=True, timeout=2.0)` would stall the camera loop for up to 2 seconds per iteration — dropping it to 0.5 fps. Threading solves this by decoupling the two loops.

---

## 1. The Pattern — Daemon Thread + Shared Dataclass

```python
from dataclasses import dataclass
import threading

@dataclass
class GpsState:
    lat: float = 10.04793
    lon: float = 76.33000
    alt: float = 6.7
    fix: bool  = False       # True when receiving real MAVLink data
```

`GpsState` is a plain Python object shared between threads. One thread writes, one thread reads — no explicit locking needed *here* because:
- Reads are single float/bool attribute accesses (atomic at CPython level due to the GIL)
- A stale GPS value from 200ms ago is completely acceptable for 5 Hz detection

If you needed sub-millisecond accuracy or were writing compound values atomically, you'd add a `threading.Lock`.

---

## 2. GpsReader Thread

```python
class GpsReader(threading.Thread):
    def __init__(self, state: GpsState, port: str, baud: int):
        super().__init__(daemon=True)   # ← thread dies when main exits
        self.state  = state
        self.port   = port
        self.baud   = baud
        self._stop  = threading.Event()

    def stop(self):
        self._stop.set()           # signal the loop to exit

    def run(self):
        if not HAS_MAVLINK:
            return                 # bail out silently if no pymavlink

        conn = mavutil.mavlink_connection(self.port, baud=self.baud)
        conn.wait_heartbeat(timeout=10)
        # ... request stream ...

        while not self._stop.is_set():
            msg = conn.recv_match(
                type=['GLOBAL_POSITION_INT'], blocking=True, timeout=2.0)
            if msg:
                self.state.lat = msg.lat / 1e7
                self.state.lon = msg.lon / 1e7
                self.state.alt = msg.alt / 1000.0
                self.state.fix = True
```

---

## 3. `daemon=True` — Why It Matters

```python
super().__init__(daemon=True)
```

A **daemon thread** is automatically killed when the main thread exits. Without this flag, pressing `Ctrl+C` would terminate the main thread but leave the GPS thread (blocked inside `recv_match`) alive, hanging the process.

Non-daemon threads must be joined explicitly:
```python
gps_thread.join()   # wait for thread to finish
```

Daemon threads skip that — perfect for "background service" threads that don't hold resources needing cleanup.

---

## 4. `threading.Event` — Clean Shutdown Signal

```python
self._stop = threading.Event()

# To stop:
gps_thread.stop()   # sets the event

# Inside the thread loop:
while not self._stop.is_set():
    ...
```

`threading.Event` is the idiomatic way to signal a background thread to exit. Alternatives (like setting a `bool` flag) work but aren't guaranteed to be seen by the other thread immediately. `Event.is_set()` is thread-safe by design.

---

## 5. Starting and Stopping

```python
# Start (non-blocking — returns immediately):
gps = GpsState()
gps_thread = GpsReader(gps, port='COM3', baud=57600)
gps_thread.start()

# ... main camera loop runs here ...

# Shutdown (Ctrl+C triggers KeyboardInterrupt):
gps_thread.stop()   # signals the thread
# daemon=True means we don't need to .join() — Python will clean it up
```

---

## 6. The GIL and Why We Don't Need Locks Here

CPython's **Global Interpreter Lock (GIL)** ensures only one thread runs Python bytecode at a time. This means that reading a single Python attribute (e.g., `gps.lat`) is atomic — you'll never see a half-written float. The trade-off is you don't get true parallelism for CPU-heavy tasks.

For this use case the GIL is actually helpful:
- GPS updates are slow (5 Hz) and simple (3 float writes)
- Camera processing is on the main thread
- No two threads are ever competing to write the same field

If you needed true parallelism for heavy image processing, you'd use `multiprocessing.Process` instead, which bypasses the GIL.

---

## stdlib — No Install Needed

`threading` is part of the Python standard library. No `pip install` required.
