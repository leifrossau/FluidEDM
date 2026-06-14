# Microspark Wire-EDM WebUI

A reworked, instrument-grade web interface for the 4-axis wire-EDM machine,
served from the FluidNC ESP32. Single self-contained `index.html` (HTML/CSS/vanilla
JS — no build step, no framework) so it gzips small and runs offline on the
machine's own WiFi AP.

## Try it
Open `index.html` in any browser. It runs a **full simulated cut** (the mock
machine) so every gauge, the spark-gap visualization, and both modes are alive
with no device attached.

## Two modes
- **Customer** (default, locked): calm full-screen view — big completion ring,
  the live spark-gap visualization, plain-language status (READY / CUTTING /
  HOLDING / COMPLETE), four health pips (Wire · Tension · Gap · Power), and
  Start / Pause / Stop. No settings, no jog. Switching to operator needs the
  4-digit code (demo: **2468**).
- **Operator**: the full instrument cluster — XYUV position, the gap-servo trace
  (advance/hold/retract + vₛ), cut-progress ring, pulse-classification bars with a
  **dashed ø rolling-average marker** on each, a **Process statistics** panel (cut
  time, total pulses, per-class averages, short events, retractions, energy delivered,
  avg feed / peak tension), wire feed + tension gauge, PSU power telemetry, an
  XYUV jog pad, job + EDM energy-mode controls, tension setpoint, touch-off,
  reset-wire-feed, and an event log.

## Smoothing
Telemetry updates ~10–12×/s. The spark-gap classifier mix and the classifier bar
fills are eased (EWMA + CSS transitions) so nothing flickers; the operator spark
canvas uses a `ResizeObserver` so it's pixel-sharp the moment the page is shown
(it's `display:none` at load, which previously left it low-res).

## Microspark Workbench integration
A header chip shows whether the **Microspark Workbench** desktop app is connected to
the same link. When a program is sent or a control is activated *from the Workbench*,
the UI raises a **toast notification** and flashes the chip. Contract: the firmware
adds `"workbench":{"connected":…}` to the telemetry and broadcasts
`{"type":"edm_event","src":"workbench","kind":…,"detail":…}` per Workbench action
(see the build note at the foot of `index.html`). The browser demo simulates the
Workbench connecting, sending programs, and tweaking controls so the feature is live.

## Signature element
The **spark-gap visualization** renders the abstract `EdmReport` classifier mix
as a living gap between the wire and the workpiece: bright cyan-white discharges
when cutting normally, clustered amber on arcing, a red bridge on a short, dim and
sparse when the gap is open — and it goes dark on a wire collapse. It turns
firmware telemetry into something an operator (or a customer) can *read at a glance*.

## Firmware binding
The page is protocol-ready against the frozen P1 `EdmReport` contract:
- **Telemetry in:** FluidNC pushes `[MSG:JSON:edm_update]{…EdmReport…}` over the
  WebSocket at `report_hz`; `FluidEdmClient` parses it. Fields consumed:
  `controller_state, servo_state, v_cmd_um_s, progress_pct, n_normal/arc/short/open,
  tension_N/tension_set_N, feed_mm_min, feed_cap_pct, wire_break_sev,
  tension_collapse, energy_uj, dc_link_v, peak_i_a, temp_gan_c, x/y/u/v, fault`.
- **Commands out:** G-code + `[ESPxxx]` + `$EDM/TouchOff`, `$EDM/Tension=`,
  `$EDM/ResetWireFeed`, `$Job/Run=`, jog `$J=`, `!`/`~`/`M3 S`/`M5`.
- When opened without a host (file:// or no socket) it transparently falls back to
  the mock machine, so it's always demoable.

Wiring the firmware side (the `[MSG:JSON:edm_update]` push + the `$EDM/*` commands
through `EdmReportChannel`) is **phase P4**. The UI already speaks that protocol.

## Deploy on-device
1. **Fonts offline:** subset Chakra Petch + IBM Plex Mono/Sans to woff2 and inline
   them as `@font-face` (data: URLs), then remove the Google Fonts `<link>` so the
   UI needs no internet on the machine's AP. (The fallback stack keeps it legible
   meanwhile.)
2. `gzip -9 -k index.html` → `index.html.gz`.
3. Upload `index.html.gz` to the ESP32 LittleFS root (WebUI uploader or `/files`).
   FluidNC serves it at `http://<machine-ip>/`.
