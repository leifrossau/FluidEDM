# Microspark EDM Pulse Supply — Board Design (MS-EDM-PSU)

> ⚠️ **HIGH-VOLTAGE, HIGH-ENERGY DESIGN — NOT FABRICATION-READY AND NOT BENCH-SAFE AS-IS.**
> This is an AI-generated architecture/first-pass scaffold for review only. An EDM pulse
> supply switches large currents at 60–120 V into a spark gap; it can destroy parts, start
> fires, and injure people. **Do not fabricate, power, or connect this to a machine until a
> qualified power-electronics engineer has reviewed the schematic, you have SPICE-simulated
> the pulse + sense paths, and you have bench-validated each stage behind current limiting.**
> Every device, creepage, and isolation rating below is a starting proposal, not a verified value.

## 1. Purpose
The board that actually generates the EDM spark for the wire cutter and reports the gap
state to the PiBot over CAN-FD. It implements the PSU side of the frozen firmware contract
(the PSU CAN catalog: `SET_MODE_BOUNDS` 0x010, `CONTROL` 0x020, `STATS_AGG` 0x100,
`PSU_STATUS` 0x300, `kProtocolVersion=1` — see the PSU integration design spec). It:
- receives gap-servo mode bounds + start/stop/clear-fault commands,
- produces controlled discharge pulses (on-time / off-time / peak current per mode),
- senses gap voltage and current every pulse and **classifies** each as normal / arc /
  short / open, and aggregates ratios into the `STATS_AGG` telemetry the gap servo consumes,
- emits status + faults; enforces hard local safety (over-current, over-temp, gap short).

## 2. Scope decisions [REVIEW] — read these first
| # | Decision | Rationale | Risk if wrong |
|---|----------|-----------|---------------|
| S1 | **Take an EXTERNAL isolated HV DC bus (60–120 VDC) as input** — this board does NOT rectify mains | Removes mains rectification, PFC, and the bulk of the lethal-creepage problem from this PCB; a COTS isolated DC supply is safer and certifiable | Still high energy; bus caps store dangerous energy |
| S2 | **Controller = STM32G474** (HRTIM, 4× fast ADC, analog comparators, FDCAN) | The pulse timing + per-pulse classification needs hardware PWM (HRTIM) and ns-class comparators; ESP32 cannot do this. Native FDCAN matches the contract | — |
| S3 | **Pulse switch = SiC/Si N-MOSFET half-bridge-style low-side switch + series current-limit + freewheel** | Standard transistor-type EDM generator; SiC handles fast edges + voltage | Device SOA / di-dt must be verified |
| S4 | **Full galvanic isolation between HV power domain and logic/CAN** (digital isolators + isolated gate driver + isolated current/voltage sense) | The PiBot/CAN side must never share the HV ground; safety + noise immunity | Isolation gap/rating is safety-critical |
| S5 | **Gap voltage sense = isolated, divided; gap current = isolated shunt/CT amp** | Per-pulse classification needs fast, isolated analog | Sense bandwidth gates classifier accuracy |
| S6 | **Discrete current-limit resistor bank per mode is OFF this first scaffold** (modeled as one limiter) | Mode-dependent peak current is set by the firmware via switched resistors / a controlled supply — a real design choice to make later | — |
| S7 | **4-layer board minimum**, HV pour on its own layer/region with explicit keep-out to logic | High di/dt loops want tight layout + plane returns; HV creepage demands separation | Layout-dependent EMI / arcing |

## 3. Block diagram
```
 HV DC IN (60-120V, external isolated supply)
   │
   ├─ bulk + film caps ─┬─ CURRENT LIMIT ─ SiC MOSFET SWITCH ──► GAP+ (wire)
   │                    │        ▲                    │
   │                    │   isolated gate drive   freewheel/clamp
   │                    │        ▲                    │
   │                 GAP V sense (iso)          GAP I sense (iso shunt)──► GAP- (work)
   │                    │                            │
 ===================  ISOLATION BARRIER  ============================
   │                    ▼                            ▼
 logic 3V3 ── STM32G474 ──┬─ HRTIM ► gate drive (across barrier)
                          ├─ ADC/COMP ◄ iso V/I sense  ► classifier (normal/arc/short/open)
                          ├─ FDCAN ─ transceiver ─ CANH/CANL ─► PiBot
                          ├─ temp sense (NTC on MOSFET/heatsink)
                          └─ hard-fault: OC comparator → gate-disable latch (HW, not FW)
```

## 4. Pulse / classification basics (firmware contract mapping)
| Contract (`STATS_AGG`) | Hardware source |
|------------------------|-----------------|
| n_normal / n_arc / n_short / n_open | per-pulse classification: gap V high+no current=open; V collapses with controlled current=normal; low-V early ignition=arc; ~0 V=short |
| window aggregation | STM32 counts pulses per telemetry window, sends ratios |
| `SET_MODE_BOUNDS` (on/off-time, peak I) | HRTIM period/duty + current-limit setpoint |
| `CONTROL` start/stop | enable/disable the gate-drive latch |
| over-current / over-temp faults | HW comparator latch + NTC; reported in `PSU_STATUS` |

## 5. Major blocks & representative parts [REVIEW]
- **HV input:** screw terminal, bulk electrolytic + film (X-rated for ripple), bleeder resistor, HV TVS/MOV, fuse.
- **Switch stage:** SiC MOSFET (e.g. 650 V class) in a TO-247/D2PAK, gate resistor network, freewheel diode (SiC Schottky), snubber (RC).
- **Isolated gate drive:** isolated gate driver (e.g. UCC21520-class) + isolated DC-DC for gate supply.
- **Sense:** current shunt + isolated amplifier (AMC1300-class); gap-voltage divider + isolated amp/ADC.
- **Controller:** STM32G474 (LQFP48/64), crystal, SWD header, FDCAN transceiver (TJA1051), CAN screw terminal + term jumper.
- **Protection:** fast OC comparator → RS-latch → gate disable (independent of firmware); NTC over-temp; status LEDs.
- **Power:** logic side from a small isolated DC-DC off the HV bus OR a separate logic supply input (recommended: separate 5 V logic input so logic can run with HV off for bring-up).

## 6. Connectors (proposed)
| Ref | Function |
|-----|----------|
| J1 | HV DC bus in (+, −) — high-current screw terminal |
| J2 | GAP output (GAP+ to wire, GAP− to workpiece) — high-current |
| J3 | CAN-FD (CANH, CANL, GND) + term jumper |
| J4 | Logic supply in (5 V, GND) — isolated from HV |
| J5 | SWD programming header (STM32) |
| J6 | NTC / temp sensor input |

## 7. What this scaffold contains vs. defers
**Contains (generated PCB):** controller + FDCAN block, isolation-barrier placeholder, gate-drive
+ switch + sense footprints placed in HV vs. logic regions, connectors, board outline with a
routed-keepout split, mounting. Full net list for the LOGIC/CAN side; HV-side nets are stubbed
to named nets for review.
**Defers (needs engineering):** exact SiC part + SOA, current-limit topology per mode, snubber
values, creepage/clearance distances, isolation component ratings, thermal/heatsink design,
EMI filtering, and ALL routing. These are the items a power engineer must own.

## 7b. Schematic & ERC status (2026-06-16)
The board is now generated **schematic-first** (`generate.py`, replacing the old PCB-only
`generate_board.py`) with **real library symbols** so every pin assignment is explicit and
reviewable: STM32G474CBTx, UCC21520 isolated gate driver, AMC1300B isolated current/voltage
amps, ADuM1402 digital isolator, TJA1051 FDCAN, AMS1117, Q_NMOS (SiC placeholder), TVS, etc.
The schematic is organized into 5 functional blocks with power-port glyphs and a title block
(`MS-EDM-PSU.sch.pdf`). **ERC: 0 errors.** Documented benign warnings:
- **3× `ground_pin_not_ground`** — the iso amps' / digital isolator's **HV-side grounds** (U11/U12
  GND1, U13 GND2) sit on the `HV-` domain, which is *deliberately isolated* from logic `GND`. This
  warning is the isolation barrier working as intended.
- **`lib_symbol_mismatch`** — flatten-vs-`extends` cache artifact on a few generic symbols
  (Q_NMOS_GSD, D_TVS, D_Schottky, C_Polarized); zero effect on netlist/fab; "Update Symbols from
  Library" reconciles in the GUI.

Still **[REVIEW]** and **deferred to a power-electronics engineer**: the isolated gate-drive +
sense supplies (`GDRV_VDD`, `ISO_HV_VDD` are PWR_FLAG'd as external/deferred), SiC part + SOA,
snubber values, gap-voltage divider ratio (`Rvd1/Rvd2`), creepage/clearance, thermals, and **all
routing**. STM32 peripheral pin choices (HRTIM→PA8, ADC→PA0-3, FDCAN→PB8/9, SWD→PA13/14) are
plausible defaults to confirm against the final firmware. **This board is NOT autorouted and NOT
fabrication-ready** — see the warning banner at the top.

## 8. Bring-up safety order (when hardware exists)
1. Logic + CAN only (no HV) → confirm contract handshake with the PiBot using firmware `SimPsuLink` parity.
2. HV bus at LOW voltage through a current-limited bench supply, no gap → verify switch + sense + OC latch.
3. Dummy resistive "gap" → verify classification math vs. scope.
4. Real wire/work gap, lowest energy mode, behind interlocks → first controlled sparks.
