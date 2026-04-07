# 07 — CSV Logging (`csv` — stdlib)

## Why CSV?

Detection hotspots need to be:
- Human-readable (open in Excel / Google Sheets immediately)
- Machine-parseable (import into QGIS, pandas, etc.)
- Appendable without loading the whole file (continuous drone flights can log thousands of entries)

CSV ticks all three boxes. The `csv` stdlib module handles quoting, escaping, and newline conventions correctly across Windows and Linux without any external dependencies.

---

## 1. Initialise — Write Header Once

```python
import csv, os

def init_csv(path: str):
    if not os.path.isfile(path):          # only write header if file doesn't exist
        with open(path, 'w', newline='') as f:
            csv.writer(f).writerow([
                'Timestamp', 'Detection_ID', 'Latitude', 'Longitude',
                'Altitude_m', 'GPS_Source', 'Severity', 'Severity_Score',
                'Area_px', 'Pixel_X', 'Pixel_Y', 'Status',
            ])
```

**`newline=''`** is required on Windows when using `csv.writer`. Without it, Python's universal newline translation would double the `\r\n` that `csv` already writes, producing blank lines between every row in Excel.

**`if not os.path.isfile`** means: if you restart the service on the same day it resumes appending to the existing log — the header isn't duplicated.

---

## 2. Append — One Row Per Detection

```python
def log_csv(path: str, det: Detection, det_id: int, gps: GpsState):
    with open(path, 'a', newline='') as f:    # 'a' = append mode
        csv.writer(f).writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],  # millisecond precision
            f'DET{det_id:05d}',                # zero-padded ID: DET00001
            f'{det.latitude:.8f}',             # 8 decimal places ≈ 1.1 mm precision
            f'{det.longitude:.8f}',
            f'{gps.alt:.2f}',
            'LIVE' if gps.fix else 'FALLBACK', # flag whether MAVLink was active
            det.severity,                      # MILD / MODERATE / SEVERE
            det.severity_score,                # 1 / 2 / 3
            det.area,                          # pixels
            det.pixel_x,
            det.pixel_y,
            'DETECTED',
        ])
```

**Why open/close every write?**  
Opening in `'a'` and immediately closing flushes the data to disk. If the service crashes (power cut, `kill -9`), no detections are lost — every logged row was flushed at the time it was written. Keeping the file open and calling `f.flush()` manually would also work but is more error-prone.

---

## 3. Output Schema

| Column | Type | Example | Notes |
|---|---|---|---|
| `Timestamp` | string | `2026-04-07 14:23:01.456` | Local time, ms precision |
| `Detection_ID` | string | `DET00042` | Zero-padded, monotone |
| `Latitude` | float | `10.04793794` | 8 dp ≈ 1.1 mm |
| `Longitude` | float | `76.33000537` | 8 dp ≈ 1.1 mm |
| `Altitude_m` | float | `6.70` | Metres AGL |
| `GPS_Source` | string | `LIVE` / `FALLBACK` | Was MAVLink active? |
| `Severity` | string | `MILD` / `MODERATE` / `SEVERE` | Area-based threshold |
| `Severity_Score` | int | `1` / `2` / `3` | Numeric version of above |
| `Area_px` | int | `312` | Blob area in pixels |
| `Pixel_X` | int | `327` | Centroid X in frame |
| `Pixel_Y` | int | `241` | Centroid Y in frame |
| `Status` | string | `DETECTED` | Reserved for future states |

---

## 4. Daily Log Rotation

```python
today    = datetime.now().strftime('%Y%m%d')
csv_path = os.path.join(Config.OUTPUT_DIR, f"disease_log_{today}.csv")
```

A new file is created each calendar day. Benefits:
- Files stay manageable in size (no multi-month megabyte blobs)
- Easy to archive or share a single day's survey
- Natural alignment with flight schedules

---

## 5. Reading the CSV Later

```python
# Python:
import csv
with open('disease_log_20260407.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        print(row['Latitude'], row['Longitude'], row['Severity'])

# pandas:
import pandas as pd
df = pd.read_csv('disease_log_20260407.csv')
df[df['Severity'] == 'SEVERE'][['Latitude', 'Longitude']].plot.scatter(x='Longitude', y='Latitude')

# QGIS:
# Layer → Add Layer → Add Delimited Text Layer → select CSV → X=Longitude, Y=Latitude
```

---

## stdlib — No Install Needed

`csv`, `os`, and `datetime` are all part of the Python standard library.
