# 01 — OpenCV (`opencv-python`)

## Why OpenCV?

OpenCV (Open Source Computer Vision Library) is the industry-standard library for real-time image processing. In this project it does **four separate jobs**:

| Job | Where in code |
|---|---|
| Read camera frames | `cv2.VideoCapture` in `main()` |
| Colour-space conversion & masking | `DiseaseDetector.detect()` |
| Morphological cleanup | `DiseaseDetector.detect()` |
| Debug visualisation window | `DiseaseDetector.draw_debug()` |

---

## 1. Reading Frames from Camera / Video

```python
cap = cv2.VideoCapture(0)          # 0 = first USB webcam
# OR
cap = cv2.VideoCapture("rtsp://192.168.1.10/stream1")  # IP camera
# OR
cap = cv2.VideoCapture("recording.mp4")               # video file

ret, frame = cap.read()
# ret  → bool: True if frame successfully grabbed
# frame → numpy array of shape (H, W, 3) in BGR colour order
```

**Why BGR and not RGB?**  
OpenCV historically stores pixels as Blue-Green-Red (not RGB). This matters when you hand colours to `cv2.putText` or draw annotations — always use `(B, G, R)` tuples.

---

## 2. HSV Colour Masking — The Core Detection Step

### Why convert to HSV?

RGB changes wildly with lighting. HSV (Hue-Saturation-Value) separates *colour identity* (hue) from *brightness* (value), making thresholds much more robust outdoors.

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Yellow disease: hue 15–40°, decent saturation and brightness
YELLOW_MIN = np.array([15,  80,  80])
YELLOW_MAX = np.array([40, 255, 255])

yellow_mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX)
# yellow_mask → 2D array: 255 where pixel is yellow, 0 elsewhere
```

**HSV ranges in OpenCV:**
- Hue: 0–179 (OpenCV halves the 0-360° scale to fit in uint8)
- Saturation: 0–255
- Value: 0–255

So "yellow" at ~30° is stored as hue **15** in OpenCV.

---

## 3. Morphological Operations — Cleaning Noisy Masks

Raw HSV masks have salt-and-pepper noise. Morphology fixes this.

```python
kernel_s = np.ones((3, 3), np.uint8)   # small kernel
kernel_l = np.ones((5, 5), np.uint8)   # large kernel

# OPEN = erode then dilate → removes tiny noise speckles
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel_s)

# CLOSE = dilate then erode → fills small gaps inside blobs
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel_l)
```

**Intuition:**
- OPEN kills specs smaller than the kernel
- CLOSE bridges gaps smaller than the kernel
- Running both in sequence gives clean, solid blobs

---

## 4. Sand / Soil Exclusion

Yellow sand looks similar to diseased plant tissue in HSV. We mask it out:

```python
BROWN_MIN = np.array([ 5, 10, 40])
BROWN_MAX = np.array([18, 100, 180])
brown_mask = cv2.inRange(hsv, BROWN_MIN, BROWN_MAX)

# Remove brown pixels from yellow mask (bitwise AND with NOT brown)
yellow_mask = cv2.bitwise_and(yellow_mask, cv2.bitwise_not(brown_mask))
```

---

## 5. Vegetation Context Constraint

Diseased spots only make sense *inside green vegetation*. If there's no green nearby, the "yellow" is probably sand, a jersey, or sunlight glare.

```python
green_total = int(np.sum(green_mask > 0))   # total green pixels in frame

if green_total > 1000:
    # Dilate green outward so nearby-but-not-touching yellow is accepted
    green_dilated = cv2.dilate(green_mask, kernel_l, iterations=5)
    yellow_mask = cv2.bitwise_and(yellow_mask, green_dilated)
```

---

## 6. Contour Detection & Shape Validation

```python
contours, _ = cv2.findContours(
    yellow_mask,
    cv2.RETR_EXTERNAL,      # only outermost contours
    cv2.CHAIN_APPROX_SIMPLE # compress horizontal/vertical/diagonal runs
)

for cnt in contours:
    area = cv2.contourArea(cnt)
    if area < 250:
        continue   # too small — skip noise

    bx, by, bw, bh = cv2.boundingRect(cnt)  # axis-aligned bounding box
    cx, cy = bx + bw // 2, by + bh // 2     # centroid

    # Shape filter: not too elongated, not too jagged
    aspect  = bw / bh
    perim   = cv2.arcLength(cnt, True)
    compact = (4 * math.pi * area) / (perim ** 2)  # 1.0 = perfect circle
    if not (0.25 <= aspect <= 4.0 and compact > 0.25):
        continue   # star-shaped / wire / reflection artefact
```

**Compactness (circularity):** Values near 1.0 = circle. Values near 0 = highly irregular shape. Diseased leaf patches tend to be roughly blob-shaped, so we reject things below 0.25.

---

## 7. Debug Window — 2×2 Grid

```python
# Resize panels to half size so they fit on screen
s = 0.5
ann_small = cv2.resize(annotated_frame, None, fx=s, fy=s)
yel_small = cv2.resize(yellow_bgr,     None, fx=s, fy=s)
grn_small = cv2.resize(green_bgr,      None, fx=s, fy=s)
ov_small  = cv2.resize(overlay,        None, fx=s, fy=s)

top    = np.hstack([ann_small, yel_small])   # side-by-side horizontally
bottom = np.hstack([grn_small, ov_small])
grid   = np.vstack([top, bottom])            # stack rows vertically

cv2.imshow('Arkairo Disease Detection', grid)
key = cv2.waitKey(1) & 0xFF   # 1ms poll — keeps window responsive
if key == ord('q') or key == 27:
    break
```

`waitKey(1)` must be called each iteration or the window freezes. Pressing **Q** or **Esc** returns the respective key code and breaks the loop.

---

## Install

```bash
pip install opencv-python
# headless servers (no display):
pip install opencv-python-headless
```

`opencv-python` ships pre-built wheels for Windows, macOS, and Linux — no system dependencies needed.
