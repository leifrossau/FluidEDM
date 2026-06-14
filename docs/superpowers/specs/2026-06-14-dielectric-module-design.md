# Sub-project D — Dielectric (Coolant) Module — Design Specification

- **Project:** Microspark Firmware (FluidEDM) + a new Dielectric module board
- **Spec date:** 2026-06-14
- **Status:** Approved for implementation planning
- **Companion contracts:** the PSU design (`../../../../Power Supply/docs/superpowers/specs/2026-05-04-power-supply-design.md`) — the dielectric is the module that spec reserves CAN IDs **0x700–0x7FF** for and states "will sit on the same CAN-FD bus when added"; and the EDM integration spec (`./2026-06-13-edm-psu-integration-design.md`).

---

## 0. Context — where it fits

The machine is a **multi-node CAN-FD bus**. The PiBot FluidNC ESP32 (Tier 3) already drives that bus through an external **MCP2518FD** (firmware `CanBus`/`Mcp2518fdDriver` + `PsuLink`). The dielectric subsystem becomes **another node on the same bus**, exactly as the PSU spec anticipated — *not* wired into the PiBot's scarce GPIO.

```
  PiBot ESP32 (FluidNC)                CAN-FD bus
    EdmController                          │
      ├─ PsuLink   (IDs 0x010–0x3FF) ──────●──── PSU board
      └─ DielLink  (IDs 0x700–0x7FF) ──────●──── Dielectric module  ← THIS
```

**Why a CAN node, not direct PiBot I/O** (decision, locked):
- The PiBot pin audit found native GPIO is the bottleneck (I2S shift-register steppers, MCP2518FD, HX711, wire-feed motors already consume it).
- Conductivity (deioniser) sensing needs **AC excitation** of the EC probe — a dedicated analog front-end, not an ESP32 `analogRead`.
- The pump (and chiller) are noisy, mains-adjacent, and sit next to deionised water — keep them isolated from the motion controller behind CAN-FD's differential link, the same reasoning that put the spark loop on the PSU board.
- The firmware CAN stack already exists; a `DielLink` is a near-copy of `PsuLink`, costing **zero** additional PiBot GPIO.

## 1. Goals & Non-Goals

### Goals
- A dielectric module on the CAN-FD bus that owns the coolant loop: pump/flush, deioniser, chiller, and flow / temperature / level / conductivity / filter sensing.
- **Bounded autonomy** (mirrors the PSU): the module runs local closed loops (temperature regulation, flush-pressure control) within bounds; the PiBot sets modes/setpoints at ~10 Hz and reads telemetry.
- A firmware **`DielLink`** (real `CanDielLink` + mock `SimDielLink`) so the integration is developed and host-tested **mock-first**, before the board exists.
- A **cut interlock**: the EDM controller will not start a cut, and faults a running cut, when flushing/level is not satisfied.
- Surface dielectric telemetry through the existing `EdmReport` → WebSocket path (the WebUI panel + `$EDM/Flush=` already consume this contract; it is **source-agnostic**).

### Non-Goals
- The dielectric **module board hardware + its MCU firmware** — defined here as a contract + front-end requirements, but built as a **separate project** (exactly as the PSU board is separate from this firmware). This spec's *implementation plan* covers only the **PiBot-side firmware** (the `DielLink`, interlock, telemetry, config).
- Mains-side AC (use an off-the-shelf brick / the dielectric board owns its own supply).
- Chiller refrigeration design, tank/plumbing mechanics.
- v1 deioniser is a resin cartridge with conductivity monitoring; automated resin regeneration is out of scope.

## 2. Decision Log (locked)

| # | Decision | Resolution | Rationale |
|---|---|---|---|
| 1 | Integration topology | Separate **CAN-FD node** on the existing bus (IDs 0x700–0x7FF) | PSU spec anticipated it; no PiBot GPIO; EMI/mains isolation; reuses firmware CAN stack |
| 2 | Control split | Bounded-autonomy module (local temp/pressure loops); PiBot sets mode/setpoints ~10 Hz | Mirrors the PSU's three-tier model |
| 3 | Wire format | Integer-encoded fields (mbar, cL/min, 0.1 °C, µS/cm, %) | Matches the firmware "No floats" boundary + EdmReport integer encoding |
| 4 | Firmware shape | `IDielLink` + `CanDielLink` + `SimDielLink`, mirroring `IPsuLink` | Proven pattern; mock-first host testing |
| 5 | Bus RX fan-out | One `CanBus` RX handler calls `psu.onFrame(f); diel.onFrame(f)` | Each `onFrame` already ignores non-matching IDs; no router needed |
| 6 | Cut interlock | Required-when-present: gate `startCut` on flow+level OK; fault on mid-cut loss; conductivity = warning only | EDM without flushing damages wire/part and is a hazard; conductivity affects quality, not safety |
| 7 | Module MCU | RP2040 or STM32G0 + CAN transceiver (pick at board design) | Same class as the PSU boot MCU |

## 3. Architecture

Three deliverables; **only #3 is in this plan's scope** (the firmware). #1/#2 are the contract + hardware requirements for the separate board project.

1. **Dielectric module board** (separate project): MCU + CAN transceiver (TJA1051) + pump driver (SSR or VFD) + chiller control + sensor front-ends (flow, temp, level, conductivity EC with AC excitation, filter ΔP). Runs the pulse/relay outputs and local loops; reports/obeys CAN.
2. **CAN-FD message catalog** (§4) — the PiBot↔module contract, in the reserved 0x700 range, mirroring the PSU §8 catalog style.
3. **PiBot firmware** (§5): `IDielLink`/`CanDielLink`/`SimDielLink`, the `EdmController` interlock + telemetry, `EdmReport` dielectric fields, `EdmReportChannel` emit, `EdmSpindle` wiring + `dielectric:` YAML, and `$EDM/Flush|Pump` commands.

## 4. CAN-FD message catalog (v1, IDs 0x700–0x7FF)

11-bit standard IDs; integer little-endian payloads (same convention as the PSU codec). Low rate (~10 Hz telemetry), so loading is negligible.

### 4.1 `SET_DIEL` (0x710, FluidNC → module, sporadic)
| Field | Type | Units / notes |
|---|---|---|
| `seq` | u16 | ack matching |
| `pump_on` | u8 | 0/1 |
| `flush_level` | u8 | 0=off,1=low,2=med,3=high |
| `temp_setpoint_dC` | i16 | 0.1 °C |
| `deioniser_enable` | u8 | 0/1 |
| `flags` | u16 | reserved, MUST be 0 |

Module replies `ACK_DIEL` (0x711): `seq` + clamped `pump_on/flush_level/temp_setpoint_dC` + `status` u8.

### 4.2 `DIEL_STATS` (0x720, module → FluidNC, ~10 Hz)
| Field | Type | Units |
|---|---|---|
| `window_id` | u32 | running counter |
| `pump_on` | u8 | 0/1 |
| `flush_level` | u8 | active 0–3 |
| `flush_mbar` | u16 | flush pressure, mbar |
| `flow_clpm` | u16 | flow, centilitres/min (650 = 6.50 L/min) |
| `temp_dC` | i16 | fluid temp, 0.1 °C |
| `temp_set_dC` | i16 | active setpoint, 0.1 °C |
| `conductivity_uS` | u16 | µS/cm |
| `level_pct` | u8 | tank level 0–100 |
| `filter_pct` | u8 | deioniser/filter life 0–100 |
| `flags` | u16 | bit0 chiller_on, bit1 low_flow, bit2 low_level, bit3 high_cond, bit4 filter_clog |

### 4.3 `DIEL_FAULT` (0x730, module → FluidNC)
`fault_code` u8, `severity` u8, `detail[6]`.

### 4.4 `DIEL_STATUS` (0x740, module → FluidNC, 100 ms heartbeat)
`state` u8, `fw_version` u16, `protocol_version` u16, `uptime_s` u32 — for discovery + version-check (mirrors `PSU_STATUS`).

## 5. PiBot firmware design

New code under `FluidNC/src/EDM/Diel/` (mirrors `EDM/Psu/`).

### 5.1 `IDielLink` + `CanDielLink` + `SimDielLink`
Mirror `IPsuLink`:
```cpp
class IDielLink {
public:
  virtual ~IDielLink() = default;
  virtual uint16_t setDiel(const SetDiel& s) = 0;   // returns seq
  virtual bool latestStats(DielStats& out) const = 0;
  virtual bool popEvent(DielEvent& out) = 0;
  virtual bool isConnected() const = 0;
  virtual bool protocolCompatible() const = 0;
  virtual bool present() const = 0;                  // a module exists/responds
};
```
- `CanDielLink(CanBus&)`: encodes `SET_DIEL`, decodes `DIEL_STATS`/`FAULT`/`STATUS` in `onFrame(f)` (ignores non-0x7xx IDs), tracks heartbeat → `isConnected`/`present`. Cross-core lock reused from the `EdmLock` pattern (the RX task vs the servo task), as in `CanPsuLink`.
- `SimDielLink`: in-firmware coolant simulator (the model already prototyped in the WebUI mock: pump follows cut, pressure/flow track flush level, temp warms under load and is chiller-regulated, conductivity creeps, level/filter deplete). Enables mock-first development + host tests.

### 5.2 `EdmController` interlock + telemetry
- Add an **optional** link: `void attachDielectric(EDM::diel::IDielLink* d);` (null = no module configured → no gating).
- **Ready-to-cut gate** at the `Armed → TouchOff` transition (`EdmController.cpp:123-133`): also require `dielReadyToCut()`. If a module is attached and `required`, that means `present() && flow_clpm ≥ flow_min && level_pct ≥ level_min`. If not ready by `_arm_deadline` → `enterFault(FaultReason::DielectricNotReady)`.
- **Mid-cut loss**: while `Cutting`/`Hold`/`BreakRelief` (NOT `TouchOff` — the gap-approach phase is still ramping flow and the `Armed→TouchOff` gate already required flow OK, so checking there would false-trip), if attached+required and flow/level drop below the floor → `enterFault(FaultReason::DielectricLost)` (feed-hold + `stopCut()`).
- **Conductivity** over `conductivity_warn_uS` sets a report warning flag only (no fault).
- Each `tick()`, poll `_diel->latestStats()` and populate `_report.diel_*`.
- New `FaultReason` values: `DielectricNotReady`, `DielectricLost`.

### 5.3 `EdmReport` dielectric fields (integer-encoded; mirrors the WebUI keys)
Add to `EdmReport`:
```cpp
bool     diel_present = false;
uint8_t  diel_pump_on = 0, diel_flush_level = 0, diel_level_pct = 0, diel_filter_pct = 0;
uint16_t diel_flush_mbar = 0, diel_flow_clpm = 0, diel_conductivity_uS = 0;
int16_t  diel_temp_dC = 0, diel_temp_set_dC = 0;
uint16_t diel_flags = 0;
```
`EdmReportChannel` emits the WebUI's `dielectric` object, converting: `flush_bar = flush_mbar/1000`, `flow_lpm = flow_clpm/100`, `temp_c = temp_dC/10`, `conductivity_us`, `level_pct`, `filter_pct`, `pump_on`, `flush_level`, `present`. **The WebUI panel already consumes exactly this** — no UI change.

### 5.4 `EdmSpindle` wiring + config
- `group()`: add `handler.section("dielectric", _diel_cfg)` — fields: `use_sim`, `required`, `flow_min_clpm`, `level_min_pct`, `conductivity_warn_uS`, `temp_setpoint_dC`, `default_flush_level`.
- `init()`: construct `SimDielLink` (when `use_sim`/no CAN) or `CanDielLink` over the shared `CanBus`; `_ctl->attachDielectric(_diel.get())`. When the CAN path is wired, set the bus RX handler to fan out: `bus.onReceive([psu,diel](const CanFrame& f){ psu->onFrame(f); diel->onFrame(f); })`.
- Commands (`EdmCommands.cpp`): `$EDM/Flush=Off|Low|Medium|High`, `$EDM/Pump=on|off` → `_diel->setDiel(...)`; dielectric block added to `$EDM/Status`.

### 5.5 Bus sharing
`CanBus::onReceive` holds one handler. With two links, `EdmSpindle` registers a single handler that calls **both** `psu->onFrame(f)` and `diel->onFrame(f)`; each ignores IDs outside its range (their `onFrame` switches on `f.id`). No router class is needed.

## 6. Module board front-end (requirements for the separate board project)
- **Pump**: SSR for on/off, or a small VFD/PWM driver for variable flush; flush levels map to pressure/flow targets.
- **Flow**: turbine/Hall pulse sensor → MCU pulse-count → cL/min.
- **Temperature**: DS18B20 (1-Wire) or NTC+ADC; chiller (or fan/Peltier) control to hold `temp_setpoint`.
- **Level**: float switch (min) + analog level, or ultrasonic.
- **Conductivity (deioniser)**: 2-electrode EC probe with **AC excitation** (square-wave drive + synchronous read) to avoid electrolysis; temperature-compensated → µS/cm.
- **Filter ΔP**: differential-pressure switch/sensor across the resin cartridge → filter_pct/clog flag.
- **Safety**: local low-level/low-flow cut-out independent of CAN; the module asserts `low_flow`/`low_level` flags and a `DIEL_FAULT` on hard limits.

## 7. Mock-first & testing
- **Host (x86 googletest)** — the firmware logic, no hardware:
  - `DielProtocol` codec: round-trip `SET_DIEL`/`ACK`/`DIEL_STATS`/`FAULT`/`STATUS`; field scaling; malformed frames.
  - `SimDielLink`: pump/flow/temp/conductivity/level model responds to `setDiel`.
  - `EdmController` interlock: with `SimDielLink`, `startCut` blocked until flow+level OK; mid-cut flow/level loss → `DielectricLost`; conductivity-high → warning not fault; `_report.diel_*` populated.
- **On-target bench** (when the module exists): real `CanDielLink` vs the module over CAN; the EDM dry-run refuses to spark without flushing; flush-level control changes pressure/flow; loss-of-flow mid-cut faults safely.

## 8. Risks / open questions
1. **Conductivity front-end** is the hardest analog block (AC excitation + temp compensation) — board-project concern; the firmware just consumes µS/cm.
2. **Interlock policy tuning**: flow/level floors are empirical (depend on kerf/thickness) — config-exposed, tuned on the bench.
3. **CAN ID/loading**: dielectric at 10 Hz is trivial next to the PSU's 1 kHz telemetry; confirm arbitration priority leaves PSU telemetry unaffected (PSU IDs are numerically lower → higher priority — good).
4. **"Required" default**: machines may run briefly without the module (bench). Default `required=false` when no `dielectric:` section; `true` once configured.

## 9. v1 Acceptance criteria
| # | Criterion |
|---|---|
| 1 | `DielProtocol` round-trips every catalog message in host tests; malformed frames rejected. |
| 2 | `SimDielLink` drives a plausible loop; `EdmController` blocks `startCut` until flow+level OK and faults on mid-cut loss (host tests). |
| 3 | `EdmReport.diel_*` populated; `EdmReportChannel` emits the `dielectric` object the WebUI already renders; `$EDM/Flush=`/`$EDM/Pump=` route to the link. |
| 4 | Two links share one `CanBus` with correct ID-range fan-out; PSU telemetry unaffected. |
| 5 | All new firmware host-tested ≥80% on logic modules; upstream FluidNC untouched outside `EDM/`. |
| 6 | (board project) the module holds temperature to setpoint, reports the catalog, and asserts low-flow/low-level safely. |

## 10. References
- Power Supply design spec §8 (CAN catalog style; reserved 0x700–0x7FF). 
- EDM integration spec (`PsuLink`/`EdmController`/`EdmReport` patterns this mirrors).
- WebUI `embedded/edm-webui/` — the `dielectric` telemetry object + `$EDM/Flush=` this firmware fulfils.
