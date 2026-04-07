# missions/

This folder has two purposes:

## 1. Drop your KML files here

Export your field boundary polygon from **Google Earth** or **Google My Maps** as a `.kml` file and place it here.

Then run from the project root:

```bash
python waypoint_generator.py missions/your_field.kml
```

## 2. Generated `.waypoints` files are saved here

The waypoint generator writes its output here automatically.  
The filename is: `<kml-name>_waypoints_<YYYYMMDD_HHMMSS>.waypoints`

Example output:
```
missions/
├── my_field.kml                          ← you drop this here
└── my_field_waypoints_20260407_194500.waypoints  ← auto-generated
```

## 3. Load into Mission Planner

1. Open **Mission Planner**
2. Go to **Flight Plan** tab
3. Click **Load WP**
4. Navigate to this `missions/` folder
5. Select the `.waypoints` file
6. Connect drone → **Upload**

---

> **Demo (no KML needed):** Run `python waypoint_generator.py` with no arguments to generate a mission for the built-in SOE sports-field polygon.
