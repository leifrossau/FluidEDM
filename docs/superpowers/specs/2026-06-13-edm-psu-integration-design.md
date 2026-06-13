# Sub-project B — PSU Integration & EDM Gap-Servo Control — Design Specification

- **Project:** Microspark Firmware (FluidEDM — a FluidNC fork), Tier-3 controller
- **Spec date:** 2026-06-13
- **Status:** Approved for implementation planning
- **Companion contract:** `../../../../Power Supply/docs/superpowers/specs/2026-05-04-power-supply-design.md` — the PSU subsystem design. Its **CAN-FD message catalog (§8) is the locked contract** this firmware develops against.

---

## 0. Context: where this fits

Adapting FluidEDM for a **4-axis wire-EDM machine** decomposes into three sub-projects, each with its own spec → plan → build cycle:

- **A — 4-axis WEDM kinematics & machine config:** XYUV kinematics (X/Y lower guide, U/V upper guide for taper), homing/limits, machine `config.yaml`. *(separate spec)*
- **B — PSU integration & EDM gap-servo control:** THIS SPEC. *(includes wire feed + tension, per decision §3.4)*
- **C — Reworked embedded WebUI (operator + customer mode):** built in parallel against B's telemetry contract (§7). *(separate spec)*

The machine is a three-tier system on a shared CAN-FD bus (see companion contract):

| Tier | Where | Loop | Owns |
|---|---|---|---|
| 1 | PSU FPGA | 10–100 ns | Per-pulse spark loop, classifier, wire-break predictor |
| 2 | PSU boot MCU | 1 ms | CAN-FD endpoint, telemetry aggregation, thermal/safety |
| **3** | **FluidNC ESP32 (THIS project)** | **1–10 ms** | **G-code, motion, mode selection, gap servo, wire feed, UI** |

The PSU owns the *spark*; this firmware owns *motion and process policy*. The two communicate only through the locked CAN-FD catalog.

## 1. Goals and Non-Goals

### Goals

- Speak the PSU CAN-FD contract from a PiBot FluidNC ESP32 V4.96 Pro board.
- Run a closed-loop **gap servo** that advances / holds / **retracts along the cut contour** to hold a gap setpoint derived from PSU telemetry.
- Drive a **closed-loop wire feed + tension** subsystem (two steppers, load-cell feedback) and react to wire-break-imminent events.
- Be **developable and testable mock-first** — full logic exercised on x86 and on-target with no PSU hardware, via an in-firmware PSU simulator and the PSU project's `mock-psu`.
- Keep all new code isolated under `src/EDM/` so upstream FluidNC merges stay clean.
- Expose EDM state + telemetry on a pinned interface (§7) so sub-project C builds in parallel.

### Non-Goals (explicitly out of scope)

- The per-pulse spark loop, pulse classification, wire-break *prediction* — all owned by the PSU.
- True taper kinematics (deriving UV from XY + angle + guide-span height) — owned by sub-project A. v1 assumes the program supplies XYUV poses (or UV = XY for straight cuts).
- The reworked WebUI — sub-project C. B only publishes the telemetry/command surface C consumes.
- Full PSU bitstream-upload relay — v1 implements version *query/check* only (§6.2); full upload relay is a later phase.
- CAM / toolpath generation for taper — host-side concern.

## 2. Decision Log (locked)

| # | Decision | Resolution | Rationale |
|---|---|---|---|
| 1 | Target controller | PiBot FluidNC ESP32 GRBL Laser CNC Controller V4.96 Pro (classic dual-core ESP32) | First-edition board choice |
| 2 | CAN-FD transport | External **MCP2518FD** CAN-FD controller over SPI (CS + INT) | Classic ESP32 TWAI is CAN 2.0 only (8-byte frames); PSU `STATS_AGG` is ~38 B → needs FD's 64-byte frames. Transport behind a `CanBus` abstraction so the chip is swappable |
| 3 | Bidirectional motion | **Custom EDM path-parameter servo** (parameter `s`, signed `ds/dt`) | grbl's planner is forward-only; feed-override floors at 0 (no retract). A path-parameter model gives true advance/hold/retract |
| 4 | Motion realization | Short-segment generator reusing `plan_buffer_line` + a shallow planner buffer | Reuses grbl's accel/stepping; retract = a short accel-limited reverse move; shallow buffer keeps the servo responsive |
| 5 | Short recovery | Retract along path (not feed-hold-and-wait) | Operator choice; correct EDM behaviour |
| 6 | Wire feed | **Full closed-loop axis**: 2 microstepping steppers (feed + tension), load-cell tension feedback | Operator choice; kept in B (not split out) |
| 7 | Wire-feed step generation | ESP32 **RMT** hardware pulse trains, independent of the kinematic stepper | Wire-feed motors are not Cartesian axes; RMT offloads jitter-free pulse timing |
| 8 | Load-cell front-end | **HX711** default (24-bit, ~80 SPS, 2-wire); ADS1232 as faster option | Tension mechanical bandwidth is low; HX711 is cheap and adequate |
| 9 | EDM ↔ G-code | EDM "cut mode" modal (active while EDM spindle on via M3), analogous to laser mode | Routes feed moves to the path servo while leaving rapids/positioning on the stock pipeline |
| 10 | Dev strategy | Mock-first: `SimPsuLink` (in-firmware) + PSU project's `mock-psu` over USB-CAN-FD | PSU hardware still in development |

## 3. Architecture

### 3.1 Module map (all under `FluidNC/src/EDM/` except the spindle facade)

| Module | Responsibility | Depends on |
|---|---|---|
| `Can/` — `CanBus` (interface) + `Mcp2518fdDriver` | CAN-FD transport over SPI; TX/RX queues; swappable backend | SPI, GPIO (CS, INT) |
| `Psu/` — `PsuLink` (interface) + `CanPsuLink` + `SimPsuLink` | Encode/decode the locked CAN-FD catalog; session layer; in-firmware PSU simulator | `Can/` |
| `EdmController` | State machine; maps cut params → `SET_MODE_BOUNDS`; runs the gap servo loop; reacts to events/faults | `Psu/`, `EdmMotion`, `WireFeed` |
| `EdmMotion` | Path-parameter servo: contour polyline, parameter `s`, `pose(s)`→XYUV, signed `ds/dt` → short segments | Planner/Stepper |
| `WireFeed` | Closed-loop feed (rate) + tension (load-cell PI) via RMT; wire-break reaction | Stepping/RMT, load-cell ADC |
| `Spindles/EdmSpindle` | Facade so `M3/M4/M5 + S` map to `EdmController` | `EdmController` |
| `EdmReport` | Publishes EDM state/telemetry to status reports + WebSocket (interface to C) | `EdmController` |

### 3.2 FluidNC integration contracts (verified against source)

- **Spindle facade.** `EdmSpindle` derives from `Spindles::Spindle` (`src/Spindles/Spindle.h`), implements `init() / setState() / setSpeedfromISR() / config_message()`, registered via `SpindleFactory::InstanceBuilder<EDM> registration("EDM")` in an anonymous namespace (pattern from `PlasmaSpindle.cpp:122`, `LaserSpindle.cpp:40`). `M3/M4/M5` reach it via `GCode.cpp` → `spindle->setState()`; `S` via `mapSpeed()`. The facade delegates to `EdmController` (start/stop spark, mode/energy select) — it holds **no** servo logic itself.
- **Motion.** `EdmMotion` emits short line segments through the existing `plan_buffer_line` path (`Planner.cpp`). The planner buffer is kept shallow (1–2 segments) for servo responsiveness. Feed override (`sys.set_f_override`) is **not** used for the servo (it cannot retract); it remains available for operator global trim.
- **Tasking.** A dedicated `edmServoTask` (1 ms, pinned to the non-protocol core) runs the controller; a CAN-RX task (INT-driven) decodes frames into a lock-free telemetry snapshot + event queue; `wireFeedTask` (~100 Hz) runs the tension PI. RMT handles wire-feed pulse timing in hardware.
- **Config.** New `edm:` and `wire_feed:` YAML sections via `Configuration::Configurable::group()` handlers (pattern from `PlasmaSpindle.h:41`).

### 3.3 Build phases (each independently testable)

- **P0 — Transport + protocol:** `CanBus`, `Mcp2518fdDriver`, full `PsuLink` codec, `SimPsuLink`. x86 unit tests for the codec; bench test vs `mock-psu` over a USB-CAN-FD adapter. Verify PiBot free pins. *No machine.*
- **P1 — Process controller:** `EdmController` state machine + `EdmSpindle` + mode/param mapping, driven by `SimPsuLink`. Gap-servo output logged, no motion.
- **P2 — Path-parameter servo:** `EdmMotion` wired to the gap servo. Spark-disabled dry run on real axes — verify advance/hold/retract under simulated gap state.
- **P3 — Wire feed + tension:** two-stepper closed loop + load cell + wire-break reaction.
- **P4 — Integration:** `EdmReport`, full YAML config, end-to-end vs `mock-psu`, then real PSU.

### 3.4 Note on scope

Wire feed + tension is kept **inside B** (operator decision) and built as phase P3. It integrates with the gap servo via the wire-break reaction path (§4.4, §5).

## 4. Gap servo & motion (the core)

### 4.1 `EdmMotion` — contour representation

- A cut is a parameterized polyline; arcs (G2/G3) flattened to line segments at a configured chord tolerance.
- Each waypoint is a full **4-axis pose** (lower XY + upper UV). Straight cuts: UV = XY. Tapered cuts carry distinct UV (supplied by the program in v1).
- Parameterized by arc length `s ∈ [0, L]`. `pose(s)` linearly interpolates between bracketing waypoints.

### 4.2 G-code integration

- An **EDM cut-mode** modal is active while the EDM spindle is on (`M3`), analogous to laser mode.
- While active, feed moves (`G1/G2/G3`) are appended to `EdmMotion`'s contour ring buffer instead of being planned normally. `G0` rapids/positioning use the stock pipeline.
- The ring buffer keeps a **trailing window of passed waypoints** (so `s` can reverse up to `retract_max_mm`) plus upcoming streamed waypoints.

### 4.3 Motion realization

- Each servo tick: compute pose increment `Δs` (sign + magnitude from the gap servo, clamped by accel/feed limits) and emit **one short line segment** to `pose(s+Δs)` via `plan_buffer_line`.
- Retract = a short segment toward a smaller `s` — a legitimate accel-limited reverse move; the planner/stepper handle kinematics.
- Planner buffer kept shallow (1–2 segments). EDM cut feeds are slow (mm/min), so limited top speed is acceptable.
- `s = L` → cut complete; spark off; normal program resumes.

### 4.4 Gap-control loop (`EdmController`)

- **Input:** `STATS_AGG` (1 kHz) — classifier counts (normal/arc/short/open), ignition-delay mean+stddev, gap-V recovery.
- **Servo error:** the modern analogue of "average gap voltage vs servo reference." Heuristic: high `open` ratio → gap too wide → advance; high `short` ratio → too close → retract; mostly `normal` → hold near setpoint.
- **Controller:** PI (anti-windup, deadband to avoid hunting) → **signed path velocity `v_s`**, clamped to `[−v_retract_max, +v_feed_max]`. Discrete reflexes layered on: sustained `short` for `T` ms → forced retract at `v_retract`; healthy `normal` band → advance toward setpoint feed. Setpoint is per-mode, configurable.
- **Touch-off:** at cut start, creep forward slowly until first sparks appear (normal/arc counts rise), establishing the gap, then engage the servo.
- **Output:** `v_s` → `EdmMotion` as `ds/dt`.

### 4.5 Safety interlocks

- PSU `FAULT` → immediate feed-hold + spark-off (M5) + alarm.
- Loss of PSU heartbeat (`0x300`) > threshold → fault.
- Stale telemetry > threshold → hold.
- Abrupt tension collapse (load cell → ~0) or persistent PSU `open` → emergency spark-off + hold.

## 5. Wire feed + tension (`WireFeed`)

- **Feed motor:** meters wire at a commanded linear rate (steps→mm via capstan circumference); runs continuously during a cut; open-loop rate (encoder optional later). RMT-timed.
- **Tension motor:** applies back-tension, closed-loop on the load cell. A PI loop modulates the tension-motor RMT frequency to hold the tension setpoint (back-tension control via speed differential). RMT-timed.
- **Load cell:** HX711 default (tare + scale in config); `wireFeedTask` ~100 Hz (load-cell-limited).
- **Wire-break reaction (severity-scaled, from `WIRE_BREAK_IMMINENT`):** sev 1–2 → reduce feed / trim tension / signal `EdmController` to back off energy; sev 3 → stop feed + feed-hold cut.

## 6. CAN-FD transport + `PsuLink`

### 6.1 Transport

- `CanBus` interface: `init() / send(CanFrame) / RX-callback`, 64-byte FD frames.
- `Mcp2518fdDriver`: SPI (configurable host/CS/INT), bit timing **1 Mbps arb / 5 Mbps data** per contract. INT → ISR → CAN-RX task drains the controller FIFO.

### 6.2 `PsuLink`

- Pure codec + session layer mirroring the locked catalog in **one versioned header** (IDs + field layouts + scaling: `dA`=0.1 A, `dC`=0.1 °C, `dV`=0.1 V, `mV`, `ns`, etc.).
- Methods/callbacks: `setModeBounds(mode, params)→seq` + `ACK_MODE_BOUNDS` handling; `startCut()/stopCut()` (control IDs 0x010–0x02F); decoded `StatsAgg` telemetry callback (0x100); event callbacks for `WIRE_BREAK_IMMINENT` (0x200), `FAULT` (0x201), `ARC_BURST` (0x202), `INFO` (0x210); `PSU_STATUS` heartbeat (0x300) for discovery + protocol-version check.
- **Bitstream/calibration upload (0x400–0x42F):** v1 implements `BITSTREAM_QUERY` + version compare only; full chunked upload relay deferred to a later phase.
- `CanPsuLink` (real) and `SimPsuLink` (in-firmware) share the interface. `SimPsuLink` models a plausible gap (advancing into a too-tight gap raises `short` ratio; retreating raises `open`), enabling servo development with zero hardware, including in x86 tests.

## 7. `EdmReport` — telemetry/command surface (pinned interface to sub-project C)

Aggregated EDM status object (decimated from 1 kHz to a UI-friendly rate):

- EDM state, active mode, servo error + `v_s`, **cut progress (s/L %)**
- Classifier ratios, ignition-delay stats, energy delivered
- Wire tension (measured/target), wire feed rate
- PSU temps, DC-link V/I
- Fault/event log, PSU connection + protocol version

Delivery:

1. **Periodic push** — an EDM JSON section on FluidNC's existing auto-report WebSocket channel, ~5–10 Hz.
2. **On-demand** — `$EDM/Status` for full detail / event history.

Commands (operator + UI): `M3 S<n>`/`M5` (spark + mode/energy via the modes table); `$EDM/Mode=<name>`, `$EDM/Status`, `$EDM/TouchOff`, `$EDM/Tension=<N>`.

This object + command list is the contract sub-project C consumes; it is frozen at the end of P1 so C can proceed in parallel.

## 8. Configuration (YAML)

```yaml
edm:
  can:
    spi_bus: ...           # SPI host
    cs_pin: ...
    int_pin: ...
    arb_kbps: 1000
    data_kbps: 5000
  servo:
    setpoint: ...          # target gap-state / normal-ratio
    kp: ...
    ki: ...
    deadband: ...
    v_feed_max_mm_per_min: ...
    v_retract_max_mm_per_min: ...
    retract_max_mm: ...
    short_retract_hold_ms: ...
    touchoff_feed_mm_per_min: ...
  modes:                    # maps S-word / named mode → SET_MODE_BOUNDS field set
    rough:   { freq_max_kHz: ..., on_time_max_ns: ..., off_time_min_ns: ..., peak_I_setpoint_dA: ..., peak_I_limit_hw_dA: ..., gap_V_arc_mV: ..., gap_V_short_mV: ..., ignition_timeout_us: ..., flags: ... }
    finish:  { ... }
wire_feed:
  feed_motor:    { step_pin: ..., dir_pin: ..., steps_per_mm: ..., default_rate_mm_per_min: ... }
  tension_motor: { step_pin: ..., dir_pin: ..., steps_per_mm: ... }
  tension:       { setpoint_N: ..., kp: ..., ki: ... }
  load_cell:     { type: hx711, dout_pin: ..., sck_pin: ..., tare_offset: ..., scale_counts_per_N: ... }
  wire_break:    { sev1_feed_scale: ..., sev2_tension_scale: ..., sev3_action: stop_and_hold }

spindles:
  EDM:
    tool_num: 0
```

## 9. Testing strategy (TDD, host-first)

- **x86 unit tests (P0–P1)** via `X86TestSupport`, tests written first:
  - PSU codec: round-trip every message; field scaling; clamp behaviour; malformed frames.
  - Gap-servo controller: synthetic open/normal/short telemetry → assert `v_s` sign/magnitude, deadband, anti-windup, reflexes, touch-off.
  - `EdmController` state machine: start/stop/fault/heartbeat-loss/cut-complete transitions.
  - `EdmMotion`: `s`→pose interpolation, contour buffering, retract-bound enforcement, segment generation from `v_s`.
  - `WireFeed`: tension PI convergence; wire-break responses.
  - All against `SimPsuLink`.
- **On-target HIL (P2+):** CAN vs `mock-psu` over USB-CAN-FD (real frames, timing, heartbeat discovery); spark-disabled dry run (real axes advance/hold/retract under simulated gap state); wire-feed bench (RMT rates, tension loop, wire-break reaction).
- **Integration (P4):** full lifecycle vs `mock-psu`, then real PSU. The PSU spec's acceptance cut is the joint milestone.
- Coverage target ≥ 80% on EDM logic modules. EDM code + tests isolated so upstream merges stay clean.

## 10. Risks & open questions

1. **PiBot free pins (highest risk).** Need ~11 GPIO beyond the 4 motion axes: MCP2518FD SPI (SCK/MOSI/MISO/CS/INT), HX711 (×2), two wire-feed steppers (step/dir ×2). PiBot drives 6 motors → X/Y/U/V use 4, leaving **2 motor headers repurposable for wire feed**. SPI may be shared with TMC drivers/SD. **Verify in P0; may require an expansion adapter** — potential hard constraint.
2. **Servo-error mapping is empirical.** Classifier-ratios → stable error + PI gains/deadband need **bench tuning on real sparks** (joint with PSU bring-up phases 4–5). `SimPsuLink` pre-tunes logic only.
3. **Short reversing segments into grbl's planner.** Validate no accel/numerical glitches with frequent direction changes (shallow buffer mitigates) — confirm in P2.
4. **CAN-RX latency** under FluidNC task load — pin to second core; validate 1 kHz servicing.
5. **Taper kinematics boundary with sub-project A.** v1 assumes program supplies XYUV (or UV = XY); deriving UV from XY+angle+guide-height is A's job — confirm hand-off.
6. **RMT channel availability** for wire-feed (ESP32 has 8; FluidNC may use some) — verify in P3.
7. **Deferred:** full bitstream-upload relay (v1 = version-check only) — confirm acceptable.

## 11. v1 Acceptance Criteria (sub-project B)

| # | Criterion |
|---|---|
| 1 | PSU codec round-trips every catalog message in x86 tests; malformed frames rejected safely. |
| 2 | Gap servo, driven by `SimPsuLink`, produces correct advance/hold/retract `v_s` across open/normal/short scenarios (x86). |
| 3 | On a real machine with spark disabled, axes advance/hold/**retract** along a test contour in response to simulated gap state, without planner/stepper glitches. |
| 4 | Wire-feed tension loop holds a setpoint against a real load cell; wire-break severities trigger the configured responses. |
| 5 | FluidNC discovers `mock-psu` over real CAN-FD, checks protocol version, runs a full mode→cut→stop lifecycle. |
| 6 | `EdmReport` publishes the §7 status object on the WebSocket at ~5–10 Hz; `$EDM/Status` returns full detail. |
| 7 | All EDM code isolated under `src/EDM/`; ≥80% unit-test coverage on logic modules; upstream FluidNC still merges cleanly. |

## 12. References

- Power Supply design spec (companion contract) — `../../../../Power Supply/docs/superpowers/specs/2026-05-04-power-supply-design.md`.
- FluidNC source integration points: `src/Spindles/Spindle.h` (base + factory), `src/Spindles/PlasmaSpindle.*`, `src/Spindles/LaserSpindle.*` (registration/config patterns), `src/Planner.cpp` (`plan_buffer_line`, feed override), `src/Stepper.cpp` (segment buffer), `src/Protocol.cpp` (task framework, feed hold), `src/GCode.cpp` (M3/M4/M5 + S dispatch).
- US4321451A — EDM gap servo system (advance/hold/retract on gap voltage). Classic servo reference.
- Yan & Chien (2007), Kinoshita et al. (1982) — wire-EDM monitoring & wire-break (PSU-side, context).
