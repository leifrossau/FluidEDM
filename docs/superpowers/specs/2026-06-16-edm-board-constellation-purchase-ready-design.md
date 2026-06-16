# EDM Board Constellation → Purchase-Ready — Design Specification

- **Project:** Microspark Firmware (FluidEDM) — `hardware/` board constellation
- **Spec date:** 2026-06-16
- **Branch:** `hw/boards`
- **Status:** Approved for implementation planning
- **Companion contracts:**
  - `docs/superpowers/specs/2026-06-13-edm-psu-integration-design.md` (PSU/gap-servo, CAN catalog 0x010–0x42F)
  - `docs/superpowers/specs/2026-06-14-dielectric-module-design.md` (dielectric, CAN catalog 0x710–0x740)
  - `hardware/dielectric-module/DESIGN.md`, `hardware/psu-board/DESIGN.md`

---

## 0. Mission

Make a **constellation of boards that is as close to ready-for-purchase as possible**, fully
headless on this machine (KiCad 10.0.3). "Ready for purchase" = orderable fab package
(Gerbers/drill/CPL) + orderable BOM (real LCSC part numbers) + complete documentation, with
human-readable, ERC-clean schematics. Verified by adversarial subagents.

User-locked scope decisions (2026-06-16):
- **Board scope:** full system set — identify and design the missing companion board(s).
- **Purchase-ready bar:** attempt full routing + gerbers (not just schematics/docs).
- **HV PSU board:** improve, but **keep the "requires power-electronics review before fab" gate**.
- **Parts sourcing:** optimize BOM for **JLCPCB assembly + LCSC parts**.

## 1. System architecture (where the boards fit)

The machine is a 4-axis wire-EDM on a shared **CAN-FD bus (1 Mbps arb / 5 Mbps data)**, three tiers:

| Tier | Node | MCU | Owns | CAN IDs |
|---|---|---|---|---|
| 3 | **PiBot V4.96 Pro** (COTS main controller) | classic ESP32 | G-code, motion (XYUV), gap servo, wire feed, UI, **CAN-FD master** | consumes all |
| 1+2 | **MS-EDM-PSU** | STM32G474 (+ FPGA, tier-1) | per-pulse spark, classifier, HV | 0x010/0x020/0x100/0x200–0x210/0x300/0x400–0x42F |
| node | **MS-DIEL-CTRL** | ESP32-S3 | coolant loop (pump/flush/chiller/sensors) | 0x710–0x740 |

The PiBot is purchased COTS, but the firmware (PSU-integration spec, risk #1) needs ~11 extra
GPIO on it — for the MCP2518FD CAN-FD link, HX711 load cell, and two wire-feed steppers — and
flags *"may require an expansion adapter."* **That adapter is the missing board.**

## 2. The constellation

| Board | Role | Designed here? | Routing | Fab-order target |
|---|---|---|---|---|
| **MS-DIEL-CTRL** | dielectric/coolant CAN node | yes (exists → finish) | autoroute | JLCPCB orderable |
| **MS-EDM-IFACE** | PiBot EDM expansion adapter (NEW) | yes (new) | autoroute | JLCPCB orderable |
| **MS-EDM-PSU** | HV pulse supply | yes (exists → finish) | **NOT autorouted** | review-ready only |
| PiBot V4.96 Pro | Tier-3 main controller | **no — COTS** | — | buy as module |
| TMC2209 stepsticks ×6 | motion drivers | **no — COTS** | — | buy as modules |

Explicitly **not** designed: a custom motion controller (the PiBot is locked by spec decision
#1), and an HMI board (operator UI is the browser WebUI, sub-project C).

## 3. New board — MS-EDM-IFACE (PiBot EDM expansion adapter)

### 3.1 Purpose
A daughterboard that gives the PiBot V4.96 Pro everything sub-project B needs that the PiBot
lacks: a CAN-FD front-end, a load-cell front-end, and clean wire-feed stepper breakouts —
without modifying the COTS controller.

### 3.2 Functional blocks
| Block | Parts | Nets to PiBot |
|---|---|---|
| **CAN-FD controller** | MCP2518FD (SOIC-14), 20/40 MHz xtal + 2×caps, decoupling | SPI: `SCK/MOSI/MISO/MCP_CS/MCP_INT` |
| **CAN-FD transceiver** | TJA1051T/3 (SOIC-8), split-termination + `JP1` term jumper, common-mode choke (opt), TVS | `CANH/CANL`, `MCP_TXCAN/RXCAN` |
| **Load-cell front-end** | HX711 (SOIC-16), ratiometric reference caps, RC filters on E+/E−/A+/A− | `HX_DOUT/HX_SCK`, JST to load cell |
| **Wire-feed breakout** | 2× stepper headers (step/dir/disable) buffered/level-safe from PiBot motor headers 5 & 6 | `WF_STEP/DIR/DIS`, `WT_STEP/DIR/DIS` |
| **Power** | 5 V in from PiBot (or screw term); 3.3 V LDO for MCP2518FD/HX711; bulk + decoupling | `+5V`, `+3V3`, `GND` |
| **Interconnect** | PiBot expansion/SPI/motor-header connectors (to be confirmed vs. PiBot schematic) | all of the above |
| **Status** | power + CAN-activity LEDs | — |

### 3.3 Connectors (proposed)
| Ref | Type | Function |
|---|---|---|
| J1 | header to PiBot SPI/expansion | SCK/MOSI/MISO/CS/INT/3V3/5V/GND |
| J2 | header to PiBot motor hdr 5 | wire-feed motor step/dir/dis |
| J3 | header to PiBot motor hdr 6 | tension motor step/dir/dis |
| J4 | Phoenix 3P | CAN-FD (CANH/CANL/GND) + JP1 term jumper |
| J5 | JST-XH 4P | HX711 load cell (E+, A+, A−, E−) |
| J6 | screw 2P (opt) | aux 5 V in if not powered from PiBot |

### 3.4 Confidence / verification gate
The PiBot V4.96 Pro connector pinout (which header exposes SPI, which motor headers are free,
3V3/5V availability) is **MEDIUM-confidence** and must be confirmed against the PiBot schematic.
Any unconfirmed pin is silkscreen-flagged and DESIGN.md-flagged **[VERIFY ON HARDWARE]**, mirroring
the firmware config's existing convention. Cross-reference `wire_edm_4axis.yaml` (i2so/SPI/GPIO
assignments) for consistency.

## 4. Schematic readability (the #1 ask)

Current schematics connect every pin via a **global label** ("label wall") — ERC-valid but
unreadable. Upgrade the `scripts/kisch.py` emitter + each `generate*.py` so output reads like a
hand-drawn sheet:

1. **Power-port symbols** (`power:+3V3`, `+5V`, `+24V`, `GND`, `PWR_FLAG`) replace power global labels.
2. **Real wires** within a functional block (e.g. decoupling cap → IC pin, divider chains, gate
   networks) instead of label-per-pin; labels kept only for genuine inter-block nets.
3. **Functional-block grouping** with frame/box + block title text (Power, MCU, CAN, Sensors, Drives…).
4. **No-connect flags** on intentionally unused pins (kills `pin_not_connected` ERC noise).
5. **`PWR_FLAG`** on each supply net (kills `power_pin_not_driven`).
6. **Symbol-library reconciliation** so embedded symbols match the cache (kills `lib_symbol_mismatch`).
7. Proper **title block** (board name, rev, date, sheet) and, where a board is large, **hierarchical sheets**.

Goal: **ERC clean (0 errors; warnings only where defensibly waived and documented)** and a PDF a
human can follow block-by-block.

## 5. Routing & manufacturing (headless)

### 5.1 Autorouting (logic boards only)
- Toolchain: pcbnew `BOARD.ExportSpecctraDSN()` → **Freerouting** (portable OpenJDK + jar installed
  to `hardware/tools/`, gitignored) headless batch → `pcbnew.ImportSpecctraSES()`.
- Pre-route: set net-class track widths/clearances (power vs. signal), keepouts, layer count
  (DIEL-CTRL 4-layer per DESIGN.md D11; IFACE 2-layer is sufficient — confirm during build).
- Post-route: fill zones, DRC. **Zone-fill segfaults headless** historically — retest on 10.0.3;
  if it still fails, fill during gerber export (`kicad-cli` fill option) or document as the single
  GUI step. Confirm gerbers contain the pours either way.

### 5.2 Manufacturing outputs (per logic board)
`kicad-cli pcb export`:
- **Gerbers** (RS-274X, all copper + mask + silk + paste + Edge.Cuts), protel ext or JLCPCB naming.
- **Excellon drill** (PTH/NPTH, map file).
- **Position/CPL** (pick-and-place, top/bottom).
- Bundle into `MS-XXX-fab.zip` per board (JLCPCB upload-ready) + a fab-notes/stackup README.

### 5.3 PSU
No autoroute. Produce readable ERC-clean schematic + verified footprints + BOM + DESIGN docs +
placement, and **preserve the explicit HV review gate**. Deliver as "review-ready," not "fab-ready."

## 6. BOM / sourcing (JLCPCB + LCSC)

Per board, emit `MS-XXX.bom.csv` in JLCPCB-friendly columns: `Comment, Designator, Footprint,
LCSC Part #, Basic/Extended, Qty, Unit Price, Description`. Every line resolves to a **real LCSC
part that exists and is in stock**, footprint-matched. Prefer **Basic** parts (no extended fee) where
a Basic part fits. Also emit a top-level **system BOM** that adds the COTS line items (PiBot
V4.96 Pro, TMC2209 modules, connectors-to-mate, load cell, sensors) with where-to-buy notes.

## 7. Documentation deliverables

Per board: `DESIGN.md` (updated, decisions still flagged [REVIEW]) + **board datasheet/ICD**
(connector pinouts, power budget, CAN IDs, indicators) + **assembly & bring-up doc**.

Repo-level:
- `hardware/README.md` rewrite — constellation overview, regenerate/route/export workflow.
- **System architecture doc** — block diagram, **full CAN-FD bus map (all node IDs)**, power tree,
  board-to-board interconnect/wiring diagram.
- **Ordering guide** — per board: fab at JLCPCB (with the zip), assemble (with the BOM), or
  review-before-fab (PSU); plus the COTS shopping list.

## 8. Verification (adversarial subagents)

After each build pass, parallel verifier agents (findings adversarially re-verified before action):
1. **ERC/DRC** — re-run via `kicad-cli`; assert 0 errors; classify/justify each remaining warning.
2. **BOM reality** — each LCSC PN exists, is in stock, footprint+value match the schematic symbol.
3. **Pinout** — each IC's net assignment vs. the datasheet pinout (MCP2518FD, TJA1051, HX711, ESP32-S3, regulators).
4. **Footprint** — KiCad footprint vs. the real package/pitch.
5. **CAN-contract** — board nets/connectors vs. the firmware CAN catalogs (0x010–0x42F, 0x710–0x740) and `wire_edm_4axis.yaml` pin map.
6. **Readability** — schematic PDF reads block-by-block; no orphan label walls.
7. **DFM** — JLCPCB capability limits (min trace/space, min drill, annular ring, layer count, board size).

## 9. Acceptance criteria

| # | Criterion |
|---|---|
| 1 | Three boards present (DIEL-CTRL, IFACE, PSU), each with a human-readable, ERC-clean schematic PDF. |
| 2 | MS-EDM-IFACE designed from scratch: MCP2518FD + TJA1051 + HX711 + wire-feed breakout + power, linked schematic+PCB. |
| 3 | DIEL-CTRL and IFACE fully routed, zones filled, DRC-clean, with JLCPCB-upload-ready fab zips. |
| 4 | Every board has a JLCPCB/LCSC BOM where every part is real + in stock; plus a system BOM with COTS. |
| 5 | PSU is readable + documented + BOM'd but retains the power-electronics-review gate (not autorouted). |
| 6 | Repo docs: README, system architecture + CAN map + interconnect, ordering guide. |
| 7 | All verifier subagents pass (or every residual is documented with justification). |
| 8 | Generators remain idempotent and re-runnable up to the point routing begins (then PCB = source of truth). |

## 10. Risks & honest limits

1. **Autorouted copper ≠ expert copper.** Orderable, but the analog conductivity front-end (DIEL-CTRL)
   and any sensitive nets warrant a human routing review — flagged, not hidden.
2. **PiBot pinout confidence.** IFACE interconnect pins are MEDIUM-confidence until checked vs. the
   PiBot V4.96 Pro schematic; unconfirmed pins flagged [VERIFY ON HARDWARE].
3. **Zone-fill headless** may still segfault → documented single GUI step if unavoidable.
4. **Freerouting install** needs network to fetch a portable JRE + jar; if unavailable, boards ship
   "placed + ratsnest + everything-but-copper" and routing becomes the one GUI step (no silent skip).
5. **PSU stays review-only** by design and safety — not a limitation to "fix."
6. **LCSC stock** fluctuates; BOM stamped with the check date, alternates listed for risky lines.

## 11. Out of scope
Custom motion controller; HMI/display board; CAM/taper kinematics; firmware changes; actual
fabrication, assembly, or powering of any board; SPICE simulation of the PSU.
