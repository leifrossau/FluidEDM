# EDM Board Constellation → Purchase-Ready — Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans / subagent-driven-development. Steps use checkbox (`- [ ]`) syntax. This is a **hardware/KiCad** plan: "tests" = ERC/DRC/verifier runs and visual PDF checks; "code" = the data-driven Python generators (`kisch.py`/`kizen.py` + per-board `generate*.py`). The KiCad-bundled python is the only interpreter that imports `pcbnew`.

**Goal:** Drive the FluidEDM board constellation (MS-DIEL-CTRL, MS-EDM-PSU, + new MS-EDM-IFACE) to as-close-to-orderable as possible headlessly: human-readable ERC-clean schematics, a new PiBot expansion board, autorouted DRC-clean logic PCBs with JLCPCB fab zips + LCSC BOMs, full docs, all subagent-verified.

**Architecture:** Data-driven generators emit both `.kicad_sch` (readable: power ports + wired blocks + NC flags) and linked `.kicad_pcb` (shared UUIDs). Autorouting via pcbnew Specctra DSN → Freerouting → SES. Fab via `kicad-cli pcb export`. PSU stays review-only (no autoroute). Verification by parallel adversarial subagents.

**Tech Stack:** KiCad 10.0.3 (`python.exe` + `pcbnew` SWIG, `kicad-cli.exe`), Freerouting (portable OpenJDK + jar), LCSC/JLCPCB part catalog.

**Conventions used below:**
- `PY` = `"C:\Program Files\KiCad\10.0\bin\python.exe"`
- `CLI` = `"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"`
- `HW` = `FluidEDM/hardware`
- Commit per task; type `hw(<board>):` or `hw(tools):`.

---

## Phase 0 — Headless toolchain (shared scripts)

### Task 0.1: Capability probe + zone-fill retest
**Files:** Create `HW/scripts/_probe.py`
- [ ] Write a probe that: imports `pcbnew`, prints version; loads each existing `.kicad_pcb`; tries `pcbnew.ZONE_FILLER(board).Fill(board.Zones())` inside a child process and reports segfault vs success; lists `BOARD.ExportSpecctraDSN`.
- [ ] Run: `PY HW/scripts/_probe.py`. Expected: confirms DSN export present; records whether zone-fill segfaults on 10.0.3.
- [ ] Commit.

### Task 0.2: Install portable Freerouting (network)
**Files:** Create `HW/tools/` (add `HW/tools/` to `HW/.gitignore`), `HW/scripts/route.py`
- [ ] Fetch a portable OpenJDK (JRE 21) + latest Freerouting CLI jar into `HW/tools/`. If network blocked, record the failure in `route.py` docstring and mark routing as the GUI fallback step (no silent skip).
- [ ] Write `route.py`: `route(pcb_path, dsn, ses, profile)` → `ExportSpecctraDSN` → run `java -jar freerouting.jar -de <dsn> -do <ses> -mp <passes>` headless → `ImportSpecctraSES`. Net-class-aware (reads widths/clearances from the board).
- [ ] Run on a throwaway copy of DIEL-CTRL. Expected: a `.ses` produced and re-imported with tracks. (Defer fill/DRC to Phase 5.)
- [ ] Commit.

### Task 0.3: Fab-export helper
**Files:** Create `HW/scripts/fab.py`
- [ ] Write `fab.py`: wraps `CLI pcb export gerbers|drill|pos` with JLCPCB-friendly options, fills zones first (via CLI fill flag or pre-fill), bundles to `<board>-fab.zip`, writes `<board>-fab/README` (stackup + fab notes). Parametrized by board dir + layer count.
- [ ] Run `CLI pcb export gerbers --help` and `drill --help` to lock exact flag names; encode them.
- [ ] Commit.

---

## Phase 1 — Schematic readability engine (`kisch.py`)

### Task 1.1: Power-port + PWR_FLAG support
**Files:** Modify `HW/scripts/kisch.py`
- [ ] Add `PowerPort(net)` emitting `power:+3V3/+5V/+24V/GND/AGND` symbols and `power:PWR_FLAG`; API to attach a power port to a pin instead of a global label.
- [ ] Verify: regenerate DIEL-CTRL (Phase-1 partial), run `CLI sch erc` — `power_pin_not_driven` count drops to 0.
- [ ] Commit.

### Task 1.2: Local wires + functional blocks + NC flags
**Files:** Modify `HW/scripts/kisch.py`
- [ ] Add `Wire(a,b)` (pin-to-pin segments + junctions), `Block(title, x,y,w,h)` (frame + text), and `NoConnect(pin)`.
- [ ] Add a block-layout helper so each `generate*.py` declares blocks (Power/MCU/CAN/Sensors/Drives) and parts/wires land inside them.
- [ ] Commit.

### Task 1.3: Title block + library reconciliation
**Files:** Modify `HW/scripts/kisch.py`
- [ ] Emit a proper title block (board, rev, date, sheet). Ensure embedded `lib_symbols` match the symbol cache (kills `lib_symbol_mismatch`).
- [ ] Commit.

### Task 1.4: Re-author DIEL-CTRL schematic for readability
**Files:** Modify `HW/dielectric-module/generate.py`
- [ ] Convert PARTS/NETS to declare blocks, power ports, intra-block wires, NC flags on unused ESP32 GPIO.
- [ ] Run: `PY HW/dielectric-module/generate.py` then `CLI sch export pdf` + `CLI sch erc --format json`.
- [ ] Gate: ERC 0 errors; every residual warning listed + justified. PDF reads block-by-block (visual check).
- [ ] Commit.

---

## Phase 2 — MS-EDM-IFACE (new PiBot expansion board)

### Task 2.1: Confirm PiBot V4.96 Pro interconnect
**Files:** Create `HW/iface-board/PINMAP.md`
- [ ] Research PiBot V4.96 Pro: which header exposes SPI (SCK/MOSI/MISO + free CS/INT), 3V3/5V rails, and which motor headers (5,6) are free. Cross-check `example_configs/wire_edm_4axis.yaml` (i2so/spi/gpio) + any `hardware-pinmap-pibot-v496pro.md`. Record confidence per pin; flag unconfirmed [VERIFY ON HARDWARE].
- [ ] Commit.

### Task 2.2: IFACE generator (schematic + linked PCB)
**Files:** Create `HW/iface-board/generate.py`, `HW/iface-board/DESIGN.md`
- [ ] Define PARTS: MCP2518FD (SOIC-14) + xtal+caps, TJA1051T/3 (SOIC-8) + split term + JP1 + TVS, HX711 (SOIC-16) + ref caps + RC filters, 3V3 LDO + decoupling, connectors J1–J6 (§3.3 of spec), 2× status LEDs.
- [ ] Define NETS by pin name vs. real library symbols (MCP2518FD SCK=10/SDI=11/SDO=12/CS=13/INT; TJA1051 TXD/RXD/CANH/CANL; HX711 PD_SCK/DOUT/E+/E-/A+/A-).
- [ ] Use the Phase-1 engine: blocks, power ports, wires, NC, title block.
- [ ] Run: `PY HW/iface-board/generate.py`; `CLI sch erc` + `sch export pdf`; `CLI pcb drc`.
- [ ] Gate: ERC 0 errors; netlist matches `wire_edm_4axis.yaml` SPI/HX711 pins; placement PDF sane.
- [ ] Write DESIGN.md (purpose, blocks, connectors, [REVIEW]/[VERIFY] flags). Commit.

---

## Phase 3 — PSU schematic + readability (keep HV gate)

### Task 3.1: Give PSU a readable schematic
**Files:** Modify `HW/psu-board/generate_board.py` → add schematic emission (or new `generate.py`)
- [ ] Port the PSU PCB-scaffold PARTS/NETS into the schematic engine (blocks: HV-in, switch/snubber, iso-gate-drive, iso-sense, STM32G474, FDCAN, protection, logic power). HV nets clearly labeled + isolation-barrier annotation. Keep [REVIEW] flags.
- [ ] Run: `PY` generate; `CLI sch erc` + `export pdf`.
- [ ] Gate: ERC 0 errors (warnings justified). DESIGN.md retains the explicit power-electronics-review gate. **No autoroute.**
- [ ] Commit.

---

## Phase 4 — BOM / LCSC sourcing (JLCPCB)

### Task 4.1: BOM emitter
**Files:** Create `HW/scripts/bom.py`
- [ ] `bom.py` reads a per-board `parts_lcsc.csv` (refdes→LCSC PN map, authored in 4.2) + the schematic, emits `MS-XXX.bom.csv` in JLCPCB columns (Comment, Designator, Footprint, LCSC, Basic/Extended, Qty, Unit Price, Description), grouped by value+footprint.
- [ ] Commit.

### Task 4.2: Resolve real LCSC parts per board (verified in Phase 8)
**Files:** Create `HW/<board>/parts_lcsc.csv` for DIEL-CTRL, IFACE, PSU
- [ ] For each unique part, pick a real in-stock LCSC PN (prefer Basic). Record value, footprint, PN, B/E, price, stock-check date, alternate.
- [ ] Run `bom.py` for each board. Gate: every line has a PN; (reality-check is Phase 8).
- [ ] Commit.

### Task 4.3: System BOM (COTS)
**Files:** Create `HW/SYSTEM-BOM.md`
- [ ] Table: PiBot V4.96 Pro, 6× TMC2209 modules, USB-CAN-FD adapter, load cell, sensors (DS18B20, pressure, flow, level/float), connectors-to-mate, wire-feed steppers — with where-to-buy + qty.
- [ ] Commit.

---

## Phase 5 — Routing (logic boards only)

### Task 5.1: Net classes + design rules
**Files:** Modify each logic board's `generate.py` (DIEL-CTRL, IFACE)
- [ ] Add net-class table: power nets wider (e.g. 0.6–1.0 mm), signal default (0.25 mm), clearance per JLCPCB (≥0.127 mm but use 0.2 mm). Layer count: DIEL-CTRL 4-layer, IFACE 2-layer (confirm). Set in `.kicad_pro`/board.
- [ ] Commit.

### Task 5.2: Autoroute + fill + DRC
**Files:** uses `HW/scripts/route.py`, `_probe` fill path
- [ ] Run `route.py` on DIEL-CTRL and IFACE; fill zones; `CLI pcb drc --format json`.
- [ ] Gate: 0 unconnected; DRC 0 errors (warnings justified). If Freerouting unavailable, mark routing as the documented GUI step (no silent skip).
- [ ] Commit routed boards.

---

## Phase 6 — Manufacturing outputs

### Task 6.1: Fab zips for logic boards
**Files:** uses `HW/scripts/fab.py`
- [ ] Run `fab.py` for DIEL-CTRL + IFACE → `*-fab.zip` (gerbers+drill+CPL) + fab-notes README (stackup, layer map, JLCPCB order settings).
- [ ] Gate: open the zip listing; confirm all copper/mask/silk/paste/edge + drill + 2 CPL files present.
- [ ] Commit (zips may be gitignored; keep the fab-notes + a checksum manifest in-repo).

### Task 6.2: PSU review package
- [ ] Export PSU placement + schematic PDF + (unrouted) board; no gerbers. Note "routing owned by power engineer."
- [ ] Commit.

---

## Phase 7 — Documentation

### Task 7.1: Per-board datasheet/ICD + bring-up
**Files:** Create `HW/<board>/DATASHEET.md` + `HW/<board>/BRINGUP.md` for all three
- [ ] Datasheet: connector pinouts, power budget, CAN IDs, indicators, mechanical (outline/mounting). Bring-up: power-on order, smoke test, per-block checkout (PSU bring-up mirrors its DESIGN §8).
- [ ] Commit.

### Task 7.2: Repo-level docs
**Files:** Rewrite `HW/README.md`; create `HW/SYSTEM-ARCHITECTURE.md`, `HW/ORDERING.md`
- [ ] SYSTEM-ARCHITECTURE: block diagram, **full CAN-FD bus map (0x010–0x42F PSU, 0x710–0x740 diel, PiBot master)**, power tree, board-to-board interconnect/wiring diagram.
- [ ] ORDERING: per board → fab@JLCPCB (zip) / assemble (BOM) / review-before-fab (PSU); + COTS shopping list (links to SYSTEM-BOM).
- [ ] README: constellation overview + regenerate/route/export workflow.
- [ ] Commit.

---

## Phase 8 — Verification (parallel adversarial subagents)

### Task 8.1: Dispatch verifier workflow
- [ ] Run a Workflow fanning out one verifier per dimension, each finding adversarially re-checked:
  1. **ERC/DRC** — re-run `CLI sch erc` + `pcb drc` all boards; assert 0 errors; classify warnings.
  2. **BOM reality** — every LCSC PN exists + in stock + footprint/value match (web check).
  3. **Pinout** — each IC net vs. datasheet (MCP2518FD, TJA1051, HX711, ESP32-S3, STM32G474, regs).
  4. **Footprint** — KiCad footprint vs. real package/pitch.
  5. **CAN-contract** — nets/connectors vs. firmware catalogs + `wire_edm_4axis.yaml`.
  6. **Readability** — schematic PDFs read block-by-block; no label walls.
  7. **DFM** — JLCPCB limits (trace/space/drill/annular/layers/size).
- [ ] Collect findings; fix confirmed issues; re-verify until clean or each residual documented.
- [ ] Commit fixes.

### Task 8.2: Final gate vs. acceptance criteria
- [ ] Walk spec §9 acceptance criteria 1–8; confirm each with evidence (ERC/DRC output, zip listing, BOM, PDFs). Write `HW/STATUS.md` summarizing done/left per board.
- [ ] Commit.

---

## Self-review notes
- **Spec coverage:** §3 IFACE→Phase 2; §4 readability→Phase 1; §5 routing/mfg→Phases 0,5,6; §6 BOM→Phase 4; §7 docs→Phase 7; §8 verification→Phase 8; §9 acceptance→Task 8.2. PSU gate preserved (Phase 3, 6.2). No gaps.
- **No placeholders:** part values/pins are concrete; LCSC PNs resolved in 4.2 and reality-checked in 8.
- **Consistency:** `route.py`/`fab.py`/`bom.py` names stable across phases; board dirs `dielectric-module`/`iface-board`/`psu-board` consistent.
