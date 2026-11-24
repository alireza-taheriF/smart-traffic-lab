# EVP-12 — Directional Emergency Vehicle Preemption

Green only for the approach the emergency vehicle is actually coming from.
Two independent directional channels must agree:

* **IR, 12 frames/s** — roof emitter (24 IR LEDs, 38 kHz carrier) → one narrow-field detector head per approach.
* **LoRa 433 MHz + GPS** — signed (HMAC-SHA256) position/heading beacon, 1 Hz → corridor geofence per approach.

The intersection unit never switches lamps: it closes a dry contact on the existing controller's
preemption input and/or sends JSON over RS-485 to the smart-traffic-lab controller.

## Contents

| Path | What |
|---|---|
| `viewer/EVP-12-3D.html` | Offline single-file 3D viewer: part labels, click-for-explanation (what / why / use), dimension sliders, exploded view, STL export per part, and an intersection simulation tab (7 scenarios) |
| `viewer/{viewer_src.html,sim.js,part_info.js,make_viewer.js}` | Viewer sources; `node make_viewer.js` rebuilds the single HTML (needs three.js r128 + JSCAD paths, see script header) |
| `cad/stl/*.stl` | 12 print-ready parts (print orientation, closed meshes), `report.json` = sizes/volumes |
| `cad/evp_parts.js` | Parametric source of every part (JSCAD) |
| `cad/build_stl.js` | `node build_stl.js [params.json] --jscad <path to @jscad/modeling>` regenerates the STLs |
| `cad/evp_openjscad.js` | Same models as one file for https://openjscad.xyz (drag & drop → parameter UI) |
| `firmware/common` | Shared protocol (IR frame codec, HMAC-SHA256, LoRa packet, decision engine) |
| `firmware/evp_tx` | Vehicle transmitter sketch (ESP32, Arduino core 3.x, libs: LoRa, TinyGPSPlus) |
| `firmware/evp_rx` | Intersection controller sketch (ESP32, Arduino core 3.x, lib: LoRa) |
| `firmware/test/host_test.cpp` | 20 host tests: crypto vectors, 5000 distorted IR frames, noise, direction scenarios |
| `tools/evp_bridge.py` | Reads the RS-485 JSON stream into smart-traffic-lab |
| `renders/` | Assembly, exploded and print-plate images |

## Quick start

```bash
# tests on a PC
cd firmware/test && g++ -std=c++17 -I../common host_test.cpp ../common/*.cpp && ./a.out
```

1. Change `FLEET_KEY` in **both** `config.h` files (32 random bytes, same on vehicles and intersections).
2. Set each vehicle's `VEHICLE_ID` / `VEHICLE_PRIO`; set each intersection's `APPROACHES` (stop-line lat/lon, inbound bearing).
3. Flash `evp_tx` and `evp_rx` from Arduino IDE (board "ESP32 Dev Module").

Firmware status: logic verified by host tests; sketches syntax-checked against the Arduino-ESP32 3.x
API headers but **not yet compiled for or run on hardware**. Do the bench steps in the design doc first.

The `src/` folders inside each sketch are copies of `firmware/common` (Arduino compiles them automatically).

License: PolyForm Noncommercial 1.0.0 (same as smart-traffic-lab).
