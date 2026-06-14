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

## What is done vs. what's left
**Done (autonomously):** architecture + part selection, board outline, every component placed by
functional block, the complete **netlist/ratsnest** to route against, copper-pour outlines,
mounting holes, DRC + PDF render, BOM.

**Left (needs the KiCad GUI / a human):**
- **Schematic capture** — there is no Python API for `.kicad_sch`; capture from each `DESIGN.md`
  + the `netlist.csv` (fast), or work the PCB directly. This also enables ERC + "update PCB from schematic".
- **Routing** — nothing is routed (the ratsnest shows all connections). DRC's "unconnected" count is expected.
- **Pin-map verification** — IC pin assignments are best-effort and stamped "verify" in the code;
  confirm against datasheets (ESP32-S3-WROOM-1, MCP2518FD, STM32G474, isolators) during capture.
- **Placement polish** — remaining DRC items are mostly silkscreen-label overlaps + courtyard
  nudges + default drill-rule flags on the big mount/terminal holes; resolve while routing.
- **Pour fill** — zones are left unfilled (headless `ZONE_FILLER` segfaults); press **B** in pcbnew.

## Regenerate a board
```powershell
& "C:\Program Files\KiCad\10.0\bin\python.exe" hardware\dielectric-module\generate_board.py
& "C:\Program Files\KiCad\10.0\bin\python.exe" hardware\psu-board\generate_board.py
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
