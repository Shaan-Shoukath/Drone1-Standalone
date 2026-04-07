# 06 — Zero-Dependency KML Parsing (`xml.etree.ElementTree`)

## Why No External Library?

The waypoint generator is designed to have **zero `pip install` requirements**. KML files are just XML with a specific schema. Python's standard library ships `xml.etree.ElementTree` which is perfectly sufficient for parsing the small polygons exported by Google Earth / My Maps.

Popular alternatives like `fastkml`, `pykml`, or `lxml` are more featureful but would add an external dependency for minimal gain.

---

## What is KML?

KML (Keyhole Markup Language) is an XML dialect for geographic data, originally created for Google Earth. A field boundary exported from Google Earth or Google My Maps looks like this:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <Placemark>
    <name>My Field</name>
    <Polygon>
      <outerBoundaryIs>
        <LinearRing>
          <coordinates>
            76.33000,10.04793,0
            76.33054,10.04773,0
            76.33057,10.04804,0
            76.33018,10.04820,0
            76.33000,10.04793,0
          </coordinates>
        </LinearRing>
      </outerBoundaryIs>
    </Polygon>
  </Placemark>
</Document>
</kml>
```

Note: **coordinates are `lon,lat,alt`** (longitude first) — the opposite of what you might expect. This is the KML spec, and easy to miss.

---

## 1. Parsing — File or Raw String

```python
import xml.etree.ElementTree as ET
import os

def parse_kml(kml_source: str):
    if os.path.isfile(kml_source):
        tree = ET.parse(kml_source)          # parse from file
        root = tree.getroot()
    else:
        root = ET.fromstring(kml_source)     # parse from string (embedded demo)
```

The `else` branch handles the built-in `DEMO_KML_CONTENT` string so the script works offline without any file.

---

## 2. Handling Namespaces

XML namespaces are the `xmlns="..."` declarations that prefix every tag with a URI. ElementTree requires explicit namespace handling:

```python
namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

# Dynamic detection (in case the user's KML uses a different URI):
if root.tag.startswith('{'):
    import re
    m = re.match(r'\{([^}]+)\}', root.tag)
    if m:
        namespace = {'kml': m.group(1)}
```

Without this, `root.find('.//Polygon')` returns `None` because the actual tag is `{http://www.opengis.net/kml/2.2}Polygon`.

---

## 3. Finding the Coordinates Element

```python
# Try the full namespaced path first (standard KML):
paths = [
    './/kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates',
    './/kml:coordinates',       # simpler path as fallback
]
coords_elem = None
for path in paths:
    coords_elem = root.find(path, namespace)
    if coords_elem is not None:
        break

# Final fallback: no namespace (some KML exporters omit it):
if coords_elem is None:
    coords_elem = root.find('.//coordinates')
```

This three-tier search handles:
1. Standard KML with full namespace path
2. Non-standard KML where `<coordinates>` is at an unexpected nesting level
3. KML files exported without namespace declarations

---

## 4. Parsing Coordinate Text

```python
coordinates = []
for token in coords_elem.text.strip().split():
    parts = token.strip().split(',')
    if len(parts) >= 2:
        # KML format: lon,lat[,alt] → we want (lat, lon)
        lat = float(parts[1])
        lon = float(parts[0])
        coordinates.append((lat, lon))
```

The raw text looks like:
```
76.33000537139496,10.04793794706056,0  76.33054440781922,10.04773133612099,0 ...
```

`.split()` on whitespace gives individual `lon,lat,alt` tokens. `split(',')` breaks each token into components.

---

## 5. Validation

```python
if coords_elem is None:
    raise ValueError("No <coordinates> element found in KML")

if len(coordinates) < 3:
    raise ValueError("KML polygon needs at least 3 points")
```

A polygon needs at least 3 vertices (triangle) to define an area. Google Earth always closes the ring by repeating the first point — so a 4-vertex field will give 5 coordinate pairs; we keep all of them and the geometry code handles the duplicate endpoint gracefully.

---

## 6. The Embedded Demo KML

```python
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
            ...
          </coordinates>
        </LinearRing>
      </outerBoundaryIs>
    </Polygon>
  </Placemark>
</Document>
</kml>"""
```

Embedding the demo polygon as a string means:
- The script produces real output with zero setup
- Offline demos work (no internet / no file required)
- Interviewers / evaluators can run it immediately

---

## stdlib — No Install Needed

`xml.etree.ElementTree` and `re` are part of the Python standard library.
