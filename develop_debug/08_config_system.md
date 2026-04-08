# 08 — Configuration System (`configparser` + argparse)

## Why a Config System?

Different deployment scenarios need different settings:

| Mode | Hardware | Serial Port | Camera | GUI |
|------|----------|-------------|--------|-----|
| `dev` | Laptop + USB webcam | COM3 | 0 | on |
| `pi5` | Raspberry Pi 5 + CSI camera | /dev/serial0 | 0 | off |

Rather than hardcoding these in the script, `config.ini` lets operators switch profiles with `--mode`.

---

## 1. `config.ini` — Profile Definitions

```ini
[dev]
camera_source = 0
serial_port   = COM3
baud          = 115200
show_gui      = true

[pi5]
camera_source = 0
serial_port   = /dev/serial0
baud          = 57600
show_gui      = false
```

The `Config` class provides defaults; `config.ini` overrides them.

---

## 2. Loading Config in Code

```python
import configparser, os

def load_mode_config(mode: str) -> dict:
    cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.ini')
    cp = configparser.ConfigParser()
    cp.read(cfg_path, encoding='utf-8')

    if not cp.has_section(mode):
        print(f"[CFG]   Mode '{mode}' not found in config.ini — using defaults")
        return {}

    print(f"[CFG]   Loaded mode '{mode}' from config.ini")
    return dict(cp[mode])
```

`configparser` handles:
- Line continuations (`\` for multi-line values)
- Case-insensitive section/key access
- Unicode files (`encoding='utf-8'`)

---

## 3. Merging Config → CLI Overrides

```python
def _get(key, cli_val, default):
    if cli_val is not None:          # CLI flag wins
        return cli_val
    return cfg.get(key, default)     # config.ini second, defaults last

serial_port = _get('serial_port', args.port,  Config.SERIAL_PORT)
baud        = int(_get('baud',    args.baud,   Config.BAUD))
show_gui    = not args.no_gui and cfg.get('show_gui', str(Config.SHOW_GUI)).lower() != 'false'
```

Priority order: **CLI args > config.ini > Config class defaults**

---

## 4. Camera Source Special Handling

```python
raw_cam = _get('camera_source', args.cam, str(Config.CAMERA_SOURCE))
cam_source = int(raw_cam) if str(raw_cam).isdigit() else raw_cam
```

This allows:
- `camera_source = 0` → USB webcam index 0
- `camera_source = /path/to/video.mp4` → video file
- `camera_source = rtsp://192.168.1.10/stream` → IP camera stream

---

## 5. CLI Arguments

```bash
python disease_detection_service.py \
    --mode dev \
    --port COM5 \
    --baud 115200 \
    --cam 1 \
    --no-gui
```

| Flag | Purpose |
|------|---------|
| `--mode` | Config profile (dev/pi5/...) |
| `--port` | Override MAVLink port |
| `--baud` | Override serial baud rate |
| `--cam` | Override camera source |
| `--no-gui` | Disable OpenCV window |

---

## stdlib — No Install Needed

`configparser` and `argparse` are part of the Python standard library.