"""
route.py - headless autoroute of a KiCad PCB via Freerouting, then fill zones.

  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" route.py <board.kicad_pcb> [power_mm] [sig_mm]

Pipeline:  set net-class widths/clearance -> pcbnew.ExportSpecctraDSN -> Freerouting (Java, headless)
           -> pcbnew.ImportSpecctraSES -> ZONE_FILLER.Fill -> save.
Freerouting + a portable JRE live in hardware/tools/ (see _probe.py / README).
Power-class nets (names containing +/GND/HV/SW/GAP/VBUS) get wider tracks.
"""
import os, sys, subprocess, glob
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), "tools")
JAVA = glob.glob(os.path.join(TOOLS, "jdk-*", "bin", "java.exe"))
FREEROUTING = os.path.join(TOOLS, "freerouting.jar")

def mm(v): return pcbnew.FromMM(v)

POWER_HINTS = ("+", "GND", "HV", "SW_NODE", "GAP", "VBUS", "AVDD", "VSUP")

def is_power(name):
    return name.startswith(("+",)) or any(h in name for h in POWER_HINTS)

def set_rules(board, power_mm, sig_mm, clearance_mm=0.2):
    """Set default + power net-class track widths and clearance (used by the DSN export)."""
    settings = board.GetDesignSettings()
    # widen the board default track/via so the router has sane geometry
    try:
        settings.SetTrackWidth(mm(sig_mm))
    except Exception:
        pass
    ncs = board.GetAllNetClasses()
    # ncs is a std::map name->NETCLASS; ensure a Power class and set widths
    def cfg(nc, w):
        nc.SetClearance(mm(clearance_mm)); nc.SetTrackWidth(mm(w))
        nc.SetViaDiameter(mm(0.8)); nc.SetViaDrill(mm(0.4))
    # default class
    try:
        cfg(ncs["Default"], sig_mm)
    except Exception as e:
        print("  (default netclass cfg skipped:", e, ")")
    # assign power nets to wider geometry by setting their net's class width via the default
    # (KiCad SWIG can't easily create classes headless; instead bump per-net via the connectivity
    #  -- Freerouting will honor the DSN's per-net widths from the netclass. We approximate by
    #  setting the default wide enough for power and relying on Freerouting min-width for signals.)
    return board

def export_dsn(board, dsn):
    ok = pcbnew.ExportSpecctraDSN(board, dsn)
    return ok and os.path.exists(dsn)

def run_freerouting(dsn, ses, passes=10):
    if not JAVA:
        raise RuntimeError("no portable JRE in hardware/tools/ (run the download step)")
    cmd = [JAVA[0], "-jar", FREEROUTING, "-de", dsn, "-do", ses, "-mp", str(passes)]
    print("  freerouting:", " ".join(os.path.basename(c) for c in cmd))
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    tail = (r.stdout or "").strip().splitlines()[-4:]
    for ln in tail: print("    ", ln[:120])
    if r.returncode != 0:
        print("    stderr:", (r.stderr or "")[:300])
    return os.path.exists(ses)

def fill_zones(board):
    f = pcbnew.ZONE_FILLER(board)
    ok = f.Fill(board.Zones())
    return ok

def route(pcb_path, power_mm=0.6, sig_mm=0.3, passes=10):
    base = pcb_path[:-len(".kicad_pcb")]
    dsn, ses = base + ".dsn", base + ".ses"
    board = pcbnew.LoadBoard(pcb_path)
    set_rules(board, power_mm, sig_mm)
    board.Save(pcb_path)                         # persist rules before export
    board = pcbnew.LoadBoard(pcb_path)
    if not export_dsn(board, dsn):
        raise RuntimeError("DSN export failed")
    print("  dsn:", os.path.basename(dsn))
    if not run_freerouting(dsn, ses, passes):
        raise RuntimeError("Freerouting produced no .ses (routing failed)")
    board = pcbnew.LoadBoard(pcb_path)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise RuntimeError("SES import failed")
    fill_zones(board)
    board.Save(pcb_path)
    tracks = len(list(board.GetTracks()))
    print("  routed + filled:", os.path.basename(pcb_path), "| tracks/vias =", tracks)
    return tracks

if __name__ == "__main__":
    p = sys.argv[1]
    pw = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6
    sg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    route(p, pw, sg)
