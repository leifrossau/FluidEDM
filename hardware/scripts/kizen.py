"""
kizen.py - tiny headless KiCad-10 board-builder helper (pcbnew SWIG API).

Shared by the per-board generate_board.py scripts. Pure-Python, no GUI.
Run with the KiCad-bundled python:
  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate_board.py
"""
import os
import csv
import pcbnew

FP_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"


def mm(v):
    return pcbnew.FromMM(float(v))


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


class Board:
    def __init__(self, copper_layers=4):
        self.b = pcbnew.BOARD()
        self.b.SetCopperLayerCount(copper_layers)
        self.nets = {}          # name -> NETINFO_ITEM
        self.fps = {}           # ref  -> FOOTPRINT
        self._assignments = []  # (ref, pad, net) for the review table
        self._skipped = []      # (ref, pad, net) pins with no matching pad (e.g. EP)

    # ---- nets ----
    def net(self, name):
        if name not in self.nets:
            ni = pcbnew.NETINFO_ITEM(self.b, name)
            self.b.Add(ni)
            self.nets[name] = ni
        return self.nets[name]

    # ---- components ----
    def add(self, ref, value, lib, fp, x, y, rot=0, uuid=None):
        path = os.path.join(FP_ROOT, lib + ".pretty") if not lib.endswith(".pretty") \
            else os.path.join(FP_ROOT, lib)
        footprint = pcbnew.FootprintLoad(path, fp)
        if footprint is None:
            raise RuntimeError("footprint not found: %s / %s" % (lib, fp))
        footprint.SetReference(ref)
        footprint.SetValue(value)
        footprint.SetPosition(V(x, y))
        if rot:
            footprint.SetOrientationDegrees(rot)
        if uuid:  # link to the schematic symbol of the same UUID (Update-PCB-from-Sch)
            try:
                footprint.SetPath(pcbnew.KIID_PATH("/" + uuid))
            except Exception as e:
                print("  (SetPath failed for %s: %s)" % (ref, e))
        self.b.Add(footprint)
        self.fps[ref] = footprint
        return footprint

    def connect(self, ref, pad_name, net_name, soft=False):
        fp = self.fps[ref]
        ni = self.net(net_name)
        hit = False
        for pad in fp.Pads():
            if pad.GetPadName() == str(pad_name):
                pad.SetNet(ni)
                hit = True
        if not hit:
            if soft:
                self._skipped.append((ref, str(pad_name), net_name))
                return
            raise RuntimeError("pad %s not found on %s" % (pad_name, ref))
        self._assignments.append((ref, str(pad_name), net_name))

    def bus(self, net_name, *ref_pads):
        for ref, pad in ref_pads:
            self.connect(ref, pad, net_name)

    # ---- board outline ----
    def rect_outline(self, w, h, x0=0.0, y0=0.0, r=3.0):
        # rounded rectangle on Edge.Cuts
        pts = [(x0 + r, y0), (x0 + w - r, y0), (x0 + w, y0 + r),
               (x0 + w, y0 + h - r), (x0 + w - r, y0 + h),
               (x0 + r, y0 + h), (x0, y0 + h - r), (x0, y0 + r)]
        segs = [(0, 1), (2, 3), (4, 5), (6, 7)]
        arcs = [(1, 2), (3, 4), (5, 6), (7, 0)]
        for a, c in segs:
            self._seg(pts[a], pts[c])
        # corner arcs approximated as short segments (kept simple/robust)
        for a, c in arcs:
            self._seg(pts[a], pts[c])

    def _seg(self, p1, p2, layer=None):
        s = pcbnew.PCB_SHAPE(self.b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(V(*p1))
        s.SetEnd(V(*p2))
        s.SetLayer(layer if layer is not None else pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        self.b.Add(s)

    def mounting_hole(self, x, y, net_name="GND"):
        fp = self.add("MH%d" % (len([r for r in self.fps if r.startswith("MH")]) + 1),
                      "M3", "MountingHole", "MountingHole_3.2mm_M3", x, y)
        for pad in fp.Pads():
            pad.SetNet(self.net(net_name))
        return fp

    # ---- copper pour ----
    def gnd_zone(self, layer, w, h, x0=0.0, y0=0.0, net_name="GND"):
        z = pcbnew.ZONE(self.b)
        z.SetLayer(layer)
        z.SetNet(self.net(net_name))
        z.SetIsRuleArea(False)
        poly = z.Outline()
        poly.NewOutline()
        for (x, y) in [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]:
            poly.Append(mm(x), mm(y))
        z.SetLocalClearance(mm(0.3))
        self.b.Add(z)
        return z

    def fill_zones(self):
        # NOTE: pcbnew.ZONE_FILLER.Fill() segfaults in the headless interpreter
        # (no GUI board settings). Zone OUTLINES are saved and valid; fill them in
        # the GUI with the "B" key, or they fill automatically on the next edit.
        print("  (zones left unfilled - press 'B' in pcbnew to pour)")

    # ---- output ----
    def save(self, path):
        self.b.Save(path)
        return os.path.exists(path)

    def write_netlist_csv(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["ref", "pad", "net"])
            for row in sorted(self._assignments):
                w.writerow(row)

    def write_bom_csv(self, path):
        rows = {}
        for ref, fp in self.fps.items():
            val = fp.GetValue()
            fpid = fp.GetFPID().GetUniStringLibItemName()
            key = (val, fpid)
            rows.setdefault(key, []).append(ref)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["qty", "value", "footprint", "refs"])
            for (val, fpid), refs in sorted(rows.items(), key=lambda kv: kv[0][0]):
                w.writerow([len(refs), val, fpid, " ".join(sorted(refs))])

    def summary(self):
        return "footprints=%d nets=%d pad_assignments=%d" % (
            len(self.b.GetFootprints()), self.b.GetNetCount(), len(self._assignments))
