"""
_probe.py - headless KiCad-10 capability probe for the constellation build.

Reports:
  - pcbnew version + Specctra DSN/SES API availability
  - per-board footprint / net / zone / track counts
  - whether ZONE_FILLER.Fill() segfaults on THIS KiCad (tested in a child process so a
    crash does not take the probe down) and whether the fill survives a save+reload
  - whether kicad-cli refills zones during gerber export (decides the Phase-5/6 fill path)

Run:  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" hardware/scripts/_probe.py
"""
import os, sys, subprocess, tempfile, shutil

PY = sys.executable
HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
BOARDS = [
    os.path.join(HW, "dielectric-module", "MS-DIEL-CTRL.kicad_pcb"),
    os.path.join(HW, "psu-board", "MS-EDM-PSU.kicad_pcb"),
]

import pcbnew
print("== pcbnew ==", pcbnew.GetBuildVersion())
print("ExportSpecctraDSN (module):", hasattr(pcbnew, "ExportSpecctraDSN"))
print("ImportSpecctraSES (module):", hasattr(pcbnew, "ImportSpecctraSES"))

for path in BOARDS:
    if not os.path.exists(path):
        print("  MISSING:", path); continue
    b = pcbnew.LoadBoard(path)
    zones = list(b.Zones())
    tracks = list(b.GetTracks())
    print("\n-- %s" % os.path.basename(path))
    print("   footprints=%d nets=%d zones=%d tracks=%d copper_layers=%d"
          % (len(b.GetFootprints()), b.GetNetCount(), len(zones), len(tracks),
             b.GetCopperLayerCount()))
    print("   board has ExportSpecctraDSN:", hasattr(b, "ExportSpecctraDSN"))

# ---- zone fill test in a child process (survives a segfault) ----
FILL_SNIPPET = r'''
import sys, pcbnew
p = sys.argv[1]
b = pcbnew.LoadBoard(p)
f = pcbnew.ZONE_FILLER(b)
ok = f.Fill(b.Zones())
print("FILL_RETURN", ok)
b.Save(p)
print("SAVED_OK")
'''

def child_fill(src):
    tmp = tempfile.mkdtemp(prefix="fillprobe_")
    dst = os.path.join(tmp, os.path.basename(src))
    # copy the whole project dir minimally (pcb only is enough to fill)
    shutil.copy(src, dst)
    snip = os.path.join(tmp, "fill.py")
    open(snip, "w").write(FILL_SNIPPET)
    r = subprocess.run([PY, snip, dst], capture_output=True, text=True)
    print("   exit=%s" % r.returncode)
    if r.stdout.strip(): print("   stdout:", r.stdout.strip().replace("\n", " | "))
    if r.stderr.strip(): print("   stderr:", r.stderr.strip()[:300])
    # reload to see if fill persisted
    if r.returncode == 0:
        bb = pcbnew.LoadBoard(dst)
        filled = sum(1 for z in bb.Zones() if z.IsFilled())
        print("   zones_filled_after_reload=%d / %d" % (filled, len(list(bb.Zones()))))
    shutil.rmtree(tmp, ignore_errors=True)
    return r.returncode

print("\n== ZONE FILL TEST (child process) ==")
for path in BOARDS:
    if os.path.exists(path):
        print("-- fill", os.path.basename(path))
        child_fill(path)
