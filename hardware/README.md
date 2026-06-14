# Microspark EDM — Hardware (KiCad 10)

KiCad 10 projects for the two boards the firmware talks to over CAN-FD but that did not yet
exist: the **dielectric (coolant) controller** and the **EDM pulse supply (PSU)**. Both were
generated programmatically with KiCad 10's headless `pcbnew` Python API + `kicad-cli`.

> **These are AI-generated first-pass designs, NOT fabrication-ready.** Read each board's
> `DESIGN.md` (every significant choice is flagged **[REVIEW]**). The PSU board is high-voltage
> and **must not be fabricated or powered without a qualified power-electronics review,
> simulation, and staged bench bring-up** — see `psu-board/DESIGN.md`.

## Layout
```
hardware/
  dielectric-module/   MS-DIEL-CTRL — ESP32-S3 + MCP2518FD CAN node, coolant sensors/drives
  psu-board/           MS-EDM-PSU   — STM32G474 HV pulse generator + classifier (review-only)
  lib/                 (project-local symbols/footprints, if any)
  scripts/kizen.py     shared headless board-builder helper
```
Each board dir has: `DESIGN.md` (the engineering), `generate_board.py` (regenerates the PCB),
`*.kicad_pro/.kicad_pcb` (the KiCad project — open in **KiCad 10**, not 9), `*.netlist.csv`
(pad→net review table), `*.bom.csv`, and `*.placement.pdf` (a rendered view).

## Workflow: schematic-first (linked)
The **dielectric board** is generated schematic-first (option A): one shared source model
(`generate.py`, the `PARTS` + `NETS` tables) emits BOTH the `.kicad_sch` (source of truth)
and the `.kicad_pcb`, with each footprint carrying the **same UUID** as its schematic symbol —
so the two are linked and "Update PCB from Schematic" / ERC are meaningful. ICs are wired by
**pin name** resolved against the real library symbol (authoritative pinouts, e.g. MCP2518FD
SCK=10/SDI=11/SDO=12/CS=13), the ESP32 by GPIO name. Connectivity in the schematic is
**global-label-per-pin** — ERC-valid and netlist-correct, but a flat "label wall", not a
hand-drawn schematic; prettify wiring in eeschema if desired. The toolkit is `scripts/kisch.py`
(schematic) + `scripts/kizen.py` (PCB). The **PSU board** is still the earlier PCB-only scaffold.

## What is done vs. what's left
**Done (autonomously):** architecture + part selection, a **linked schematic + PCB** (dielectric),
real-symbol pinouts, board outline, block placement, complete netlist/ratsnest, copper-pour
outlines, mounting, ERC + DRC + schematic/PCB PDFs, BOM.

**Left (needs the KiCad GUI / a human):**
- **Routing** — nothing is routed (the ratsnest shows all connections). DRC's "unconnected" count is expected.
- **ERC nags** — the generated schematic leaves benign warnings: unused-GPIO `pin_not_connected`
  (add no-connect flags), `power_pin_not_driven` (add `PWR_FLAG`s), and `lib_symbol_mismatch`
  (the embedded symbols differ slightly from the library cache — "Update Symbols from Library").
- **Placement polish** — remaining DRC items are mostly silkscreen-label overlaps + courtyard
  nudges + default drill-rule flags on the big mount/terminal holes; resolve while routing.
- **A few [REVIEW] part choices** — buck `AP3402`, reverse-FET `AO3401A`, and the programming
  header replacing USB-C (pad-mapping); confirm against the BOM you intend to buy.
- **Pour fill** — zones are left unfilled (headless `ZONE_FILLER` segfaults); press **B** in pcbnew.

## Regenerate a board
```powershell
& "C:\Program Files\KiCad\10.0\bin\python.exe" hardware\dielectric-module\generate.py        # schematic + linked PCB
& "C:\Program Files\KiCad\10.0\bin\python.exe" hardware\psu-board\generate_board.py           # PCB scaffold only
```
Render / check (kicad-cli):
```powershell
$cli = "C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
& $cli pcb export pdf  --layers "F.Cu,F.Silkscreen,F.Fab,Edge.Cuts" --mode-single <board>.kicad_pcb
& $cli pcb drc --format json --output drc.json <board>.kicad_pcb
```
The generators are **idempotent** — re-running rebuilds the `.kicad_pcb` from the component +
net tables in `generate_board.py`. Edit those tables (or `scripts/kizen.py`) to change the board;
re-run to regenerate. (Once you start hand-editing/routing in the GUI, stop re-running the
generator or it will overwrite your work — at that point the `.kicad_pcb` is the source of truth.)

## Firmware contracts these implement
- **Dielectric:** CAN IDs `0x710–0x740` — `docs/superpowers/specs/2026-06-14-dielectric-module-design.md`.
- **PSU:** CAN IDs `0x010/0x020/0x100/0x300` — the PSU integration design spec.
Both boards' MCUs run separate firmware (not in this repo yet); the contracts + the firmware's
`SimPsuLink`/`SimDielLink` simulators let you validate the bus handshake before either board exists.
