# 02 — NumPy

## Why NumPy?

NumPy provides the multi-dimensional array type (`ndarray`) that every OpenCV image *is*. Without NumPy there would be no way to define HSV thresholds, manipulate pixels, or construct the 2×2 debug grid. It is also used for coordinate-math operations that would otherwise need slow Python loops.

---

## 1. Images Are Just NumPy Arrays

```python
import cv2, numpy as np

ret, frame = cap.read()
print(type(frame))         # <class 'numpy.ndarray'>
print(frame.shape)         # (480, 640, 3)  → (H, W, channels)
print(frame.dtype)         # uint8  → values 0-255
```

Every pixel operation OpenCV does under the hood is a vectorised NumPy operation — far faster than Python `for` loops.

---

## 2. Defining HSV Threshold Arrays

```python
# Without NumPy you'd have to pass plain tuples — but np.array ensures
# the correct dtype (uint8) that inRange expects.
YELLOW_MIN = np.array([15,  80,  80], dtype=np.uint8)
YELLOW_MAX = np.array([40, 255, 255], dtype=np.uint8)

yellow_mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX)
# yellow_mask.dtype  → uint8
# yellow_mask.shape  → (H, W)   — single channel (no colour dimension)
# pixel value: 255 if in range, 0 otherwise
```

---

## 3. Counting Pixels — `np.sum`

```python
green_total = int(np.sum(green_mask > 0))
```

`green_mask > 0` produces a boolean array (True/False per pixel).  
`np.sum(...)` counts Trues as 1 → total pixel count in one vectorised call, far faster than `len([p for p in mask.flat if p > 0])`.

---

## 4. Morphology Kernels

```python
kernel_s = np.ones((3, 3), np.uint8)  # 3×3 block of all 1s
kernel_l = np.ones((5, 5), np.uint8)  # 5×5 block of all 1s
```

Morphological ops convolve with these kernels. A kernel of all 1s means "treat every neighbour equally" — it's the standard structuring element for erosion/dilation.

---

## 5. Pixel Colouring via Boolean Indexing

```python
yel_bgr = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
yel_bgr[yellow_mask > 0] = [0, 220, 220]   # paint yellow blobs cyan
```

`yellow_mask > 0` selects a subset of rows/columns simultaneously — this is NumPy *fancy indexing*. Assigning a list `[B, G, R]` to that selection sets all those pixels at once.

---

## 6. Stacking Panels — `np.hstack` / `np.vstack`

```python
top    = np.hstack([panel_a, panel_b])   # concatenate along columns (width)
bottom = np.hstack([panel_c, panel_d])
grid   = np.vstack([top, bottom])        # concatenate along rows (height)
```

Both panels must have the same height for `hstack`, same width for `vstack`. That's why we resize each panel to 50% first — they all share the same dimensions.

---

## 7. ENU Coordinate Arrays

In `waypoint_generator.py`, polygon vertices are converted to ENU (East-North-Up) local coordinates stored as plain Python lists of tuples — but the underlying sin/cos math uses `math` (not NumPy) since we're operating on scalars, not arrays. If you ever need to process hundreds of polygons at once, the natural upgrade would be:

```python
import numpy as np

lats = np.array([p[0] for p in polygon])
lons = np.array([p[1] for p in polygon])
# vectorised haversine across all vertices at once
dlat = np.radians(lats[1:] - lats[:-1])
# ...
```

---

## Install

NumPy is a dependency of `opencv-python`, so it is installed automatically:

```bash
pip install opencv-python   # pulls numpy as a dependency
# or explicitly:
pip install numpy
```
