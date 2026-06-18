"""
kisch.py - minimal KiCad-10 schematic (.kicad_sch) emitter built on a tiny S-expression
parser. Reads real symbol definitions + pin geometry from the installed .kicad_sym libraries
so generated symbols are genuine (embeddable in lib_symbols) and connectivity can be placed
at true pin endpoints.

Connectivity method: each pin gets a short wire stub + a global label = its net name
(connect-by-name). ERC-valid and netlist-correct; not visually hand-drawn.

No external deps; run with the KiCad-bundled python.
"""
import os, re

SYM_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\symbols"


# ----------------------------- S-expression parser -----------------------------
def tokenize(s):
    toks = []
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c in "()":
            toks.append(c); i += 1
        elif c == '"':
            j = i + 1; buf = []
            while j < n:
                if s[j] == "\\":
                    buf.append(s[j + 1]); j += 2; continue
                if s[j] == '"':
                    break
                buf.append(s[j]); j += 1
            toks.append(("str", "".join(buf))); i = j + 1
        elif c in " \t\r\n":
            i += 1
        else:
            j = i
            while j < n and s[j] not in "() \t\r\n\"":
                j += 1
            toks.append(("atom", s[i:j])); i = j
    return toks


def parse(toks):
    pos = [0]

    def rd():
        t = toks[pos[0]]; pos[0] += 1
        if t == "(":
            lst = []
            while toks[pos[0]] != ")":
                lst.append(rd())
            pos[0] += 1
            return lst
        return t

    out = []
    while pos[0] < len(toks):
        out.append(rd())
    return out


def head(node):
    if isinstance(node, list) and node and isinstance(node[0], tuple) and node[0][0] == "atom":
        return node[0][1]
    return None


def find_all(node, name):
    res = []
    if isinstance(node, list):
        if head(node) == name:
            res.append(node)
        for ch in node:
            res.extend(find_all(ch, name))
    return res


def num(tok):
    return float(tok[1])


# ----------------------------- symbol library access -----------------------------
def lib_path(nick):
    return os.path.join(SYM_ROOT, nick + ".kicad_sym")


def raw_symbol_block(text, name):
    """Return the exact substring of the top-level (symbol "name" ...) block."""
    needle = '(symbol "%s"' % name
    start = text.find(needle)
    if start < 0:
        raise RuntimeError("symbol %s not found" % name)
    depth = 0
    i = start
    while i < len(text):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    raise RuntimeError("unbalanced symbol block for %s" % name)


def extract_pins(block_text):
    """Parse a symbol block -> list of {number,name,x,y,angle,length}."""
    node = parse(tokenize(block_text))[0]
    pins = []
    for pin in find_all(node, "pin"):
        at = next((c for c in pin if head(c) == "at"), None)
        length = next((c for c in pin if head(c) == "length"), None)
        number = next((c for c in pin if head(c) == "number"), None)
        name = next((c for c in pin if head(c) == "name"), None)
        if not at:
            continue
        pins.append({
            "number": number[1][1] if number else "?",
            "name": name[1][1] if name else "~",
            "x": num(at[1]), "y": num(at[2]), "angle": float(at[3][1]),
            "length": num(length[1]) if length else 0.0,
        })
    return pins


def _extends_target(block_text):
    m = re.search(r'\(extends "([^"]+)"', block_text)
    return m.group(1) if m else None


class SymDef:
    """A library symbol: its raw block (for embedding) + parsed pins.

    Derived symbols (`(extends "PARENT")`) are FLATTENED onto the parent's graphics +
    pins and re-named to the child's lib_id, so each embedded symbol is standalone
    (no parent-reference bookkeeping needed in lib_symbols)."""
    def __init__(self, nick, name):
        self.nick = nick
        self.name = name
        self.lib_id = "%s:%s" % (nick, name)
        text = open(lib_path(nick), encoding="utf-8").read()
        block = raw_symbol_block(text, name)
        src_name = name
        parent = _extends_target(block)
        depth = 0
        while parent and depth < 5:
            block = raw_symbol_block(text, parent)   # flatten onto parent body
            src_name = parent
            parent = _extends_target(block)
            depth += 1
        self._src_name = src_name
        self.raw = block
        self.pins = extract_pins(block)

    def pin_number(self, spec):
        """Resolve a pin spec (number OR pin name) to its pin-number string."""
        spec = str(spec)
        for p in self.pins:
            if p["number"] == spec:
                return spec
        for p in self.pins:
            if p["name"] == spec:
                return p["number"]
        raise KeyError("%s has no pin %r" % (self.lib_id, spec))

    def embed(self):
        """Raw block re-named for lib_symbols: child unit sub-symbols `<src>_u_s` ->
        `<name>_u_s`, then the top-level symbol -> full lib_id. (When the symbol was
        flattened from a parent via `extends`, <src> is the parent's bare name.)"""
        block = self.raw
        block = block.replace('(symbol "%s_' % self._src_name, '(symbol "%s_' % self.name)
        block = block.replace('(symbol "%s"' % self._src_name, '(symbol "%s"' % self.lib_id, 1)
        return block


import hashlib


def uuid_for(key):
    """Deterministic UUID from a key string (so the same ref yields the same UUID in
    both the schematic and the PCB generators -> they are linked)."""
    h = hashlib.md5(key.encode("utf-8")).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[0:8], h[8:12], h[12:16], h[16:20], h[20:32])


class Comp:
    def __init__(self, ref, value, symdef, footprint, x, y, rot=0):
        self.ref = ref
        self.value = value
        self.sym = symdef
        self.fp = footprint
        self.x = x
        self.y = y
        self.rot = rot
        self.uuid = uuid_for("sym:" + ref)

    def pin_point(self, pin):
        # symbol lib coords are Y-up; schematic sheet is Y-down. rot=0, no mirror.
        if self.rot == 0:
            return (self.x + pin["x"], self.y - pin["y"])
        raise NotImplementedError("only rot=0 supported in this generator")


# ----------------------------- power-net styling -----------------------------
# Default classification so a schematic reads like a human drew it instead of a
# "label wall": power/ground pins get POWER-PORT GLYPHS, ordinary signals get LOCAL
# net labels, and only nets explicitly marked global get the heavy global-label arrows.
_GND_SYMBOL = {            # net name -> KiCad power.kicad_sym ground glyph
    "GND": "GND", "AGND": "GNDA", "GNDA": "GNDA", "DGND": "GND",
    "PGND": "GNDPWR", "GNDPWR": "GNDPWR", "HV_GND": "GNDPWR", "EARTH": "Earth",
}
# rail glyphs are looked up by the net name itself ("+3V3", "+5V", "+24V", ...);
# any that exist in power.kicad_sym work. This set just marks which names are rails.
_RAIL_HINTS = ("+", "VCC", "VDD", "VBUS", "VIN", "VBAT")


def is_power_net(net):
    if net in _GND_SYMBOL:
        return True
    return net.startswith(_RAIL_HINTS) or net in ("VCC", "VDD")


def power_glyph(net):
    """Return the power.kicad_sym symbol name that represents `net`, or None."""
    if net in _GND_SYMBOL:
        return _GND_SYMBOL[net]
    if net.startswith("+") or net in ("VCC", "VDD"):
        return net
    return None


# Unit vector (SHEET space, +y = down) pointing AWAY from the symbol body for a pin
# at the given KiCad pin angle. 0=pin on left edge, 180=right, 90=bottom, 270=top.
_OUTWARD = {0: (-1.0, 0.0), 180: (1.0, 0.0), 90: (0.0, 1.0), 270: (0.0, -1.0)}
STUB = 2.54   # mm: pull labels/power glyphs off the pin (kept short so stacked 2-pin
              # parts at ~12 mm pitch don't have facing stubs collide; ICs get extra room
              # from placement, not stub length)


def sym_half_height(sym):
    """Vertical half-extent of a symbol's pins (mm), used for auto-spacing columns."""
    return max((abs(p["y"]) for p in sym.pins), default=2.54)


def stack_col(refs, symof, y_top, gap=3.81):
    """Place `refs` in a vertical column with guaranteed clearance between each part's
    facing pin STUB ends. Returns {ref: origin_y}. `symof(ref)` -> SymDef. `y_top` is the
    sheet y of the first part's top stub end; parts grow downward."""
    ys, prev_y, prev_h = {}, None, None
    snap = lambda v: round(v / 1.27) * 1.27
    for r in refs:
        h = sym_half_height(symof(r))
        if prev_y is None:
            y = y_top + h + STUB
        else:
            y = prev_y + prev_h + STUB + gap + h + STUB
        ys[r] = snap(y)
        prev_y, prev_h = ys[r], h
    return ys


class Schematic:
    """Readable single-sheet schematic emitter.

    Connectivity model (declared via connect/bus, classified at emit time):
      * power/ground nets   -> a power-port glyph at the pin (decluttered, conventional)
      * nets in `globals`   -> a global-label arrow (use for true off-board / cross-sheet I/O)
      * everything else      -> a local net label (KiCad joins same-named labels on the sheet)
    Plus optional functional BLOCK frames + titles, explicit wires, no-connect flags, a
    PWR_FLAG per supply net, and a title block -- the things that make a generated sheet
    navigable by a human.
    """
    def __init__(self, project, paper="A3", title=None, rev="A", date="", company=""):
        self.project = project
        self.paper = paper
        self.title = title or project
        self.rev = rev
        self.date = date
        self.company = company
        self.sheet_uuid = uuid_for("sheet:" + project)
        self.comps = []
        self.net_of = {}        # (ref, pin_number) -> net name
        self.globals = set()    # net names to render as GLOBAL labels
        self.power_nets = set() # extra net names to force-treat as power
        self.auto_nc = set()    # refs whose unconnected pins get no-connect flags
        self._blocks = []       # (title, x, y, w, h)
        self._wires = []        # (p1, p2, key)
        self._ncs = []          # (x, y, key)
        self._flags = []        # (net, x, y) PWR_FLAG anchors
        self._notes = []        # (text, x, y, size)
        self._pwr_seq = 0
        self._flg_seq = 0
        # power symbol cache (lazily filled)
        self._pwr_syms = {}

    # ---- model ----
    def add(self, comp):
        self.comps.append(comp)
        return comp

    def connect(self, ref, pin_number, net):
        self.net_of[(ref, str(pin_number))] = net

    def bus(self, net, *ref_pins):
        for ref, pin in ref_pins:
            self.connect(ref, pin, net)

    def mark_global(self, *nets):
        self.globals.update(nets)

    def mark_power(self, *nets):
        self.power_nets.update(nets)

    def nc_unused(self, *refs):
        """Auto-emit no-connect flags on every UNCONNECTED pin of these refs."""
        self.auto_nc.update(refs)

    # ---- annotations ----
    def block(self, title, x, y, w, h):
        self._blocks.append((title, x, y, w, h))

    def wire(self, p1, p2):
        self._wires.append((p1, p2, "%s-%s" % (p1, p2)))

    def power_flag(self, net, x, y):
        self._flags.append((net, x, y))

    def note(self, text, x, y, size=1.27):
        self._notes.append((text, x, y, size))

    # ---- power-net classification ----
    def _is_power(self, net):
        return net in self.power_nets or is_power_net(net)

    def _power_sym(self, glyph):
        if glyph not in self._pwr_syms:
            self._pwr_syms[glyph] = SymDef("power", glyph)
        return self._pwr_syms[glyph]

    # ---- emit: symbol instance ----
    def _symbol_instance(self, c):
        # Reference/Value placement: tall ICs get them ABOVE/BELOW the body (clear of the
        # side pin labels); small 2-pin parts keep them to the right (their sides are open).
        hh = sym_half_height(c.sym)
        if hh >= 5.0:
            rx, ry, jr = c.x + 3.5, c.y - hh - 6.0, "left"   # offset clear of the centred VDD/VSS glyph
            vx, vy, jv = c.x + 3.5, c.y + hh + 6.0, "left"
        else:
            rx, ry, jr = c.x + 6.0, c.y - 2.0, "left"
            vx, vy, jv = c.x + 6.0, c.y + 2.0, "left"
        L = []
        a = L.append
        a('\t(symbol')
        a('\t\t(lib_id "%s")' % c.sym.lib_id)
        a('\t\t(at %g %g %d)' % (c.x, c.y, c.rot))
        a('\t\t(unit 1)')
        a('\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)')
        a('\t\t(uuid "%s")' % c.uuid)
        a('\t\t(property "Reference" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (justify %s)))' % (c.ref, rx, ry, jr))
        a('\t\t(property "Value" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (justify %s)))' % (c.value, vx, vy, jv))
        a('\t\t(property "Footprint" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (hide yes)))' % (c.fp, c.x, c.y))
        a('\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))'
          % (self.project, self.sheet_uuid, c.ref))
        a('\t)')
        return "\n".join(L)

    # ---- emit: power-port instance (a power.kicad_sym glyph whose Value = net) ----
    def _power_port(self, net, x, y):
        glyph = power_glyph(net)
        if glyph is None:
            return self._local_label(net, x, y, "pp:%s:%g:%g" % (net, x, y))
        try:
            sym = self._power_sym(glyph)
        except Exception:
            return self._local_label(net, x, y, "pp:%s:%g:%g" % (net, x, y))
        self._pwr_seq += 1
        ref = "#PWR%02d" % self._pwr_seq
        uid = uuid_for("pwr:%s:%g:%g" % (net, x, y))
        # label position: rails above (glyph points up), grounds below (glyph points down)
        is_gnd = net in _GND_SYMBOL
        return ('\t(symbol\n'
                '\t\t(lib_id "power:%s")\n'
                '\t\t(at %g %g 0)\n'
                '\t\t(unit 1)\n'
                '\t\t(exclude_from_sim no) (in_bom no) (on_board yes) (dnp no)\n'
                '\t\t(uuid "%s")\n'
                '\t\t(property "Reference" "%s" (at %g %g 0)\n'
                '\t\t\t(effects (font (size 1.27 1.27)) (hide yes)))\n'
                '\t\t(property "Value" "%s" (at %g %g 0)\n'
                '\t\t\t(effects (font (size 1.27 1.27))))\n'
                '\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))\n'
                '\t)' % (glyph, x, y, uid, ref, x, y - 5,
                        net, x, (y + 4.0 if is_gnd else y - 4.0),
                        self.project, self.sheet_uuid, ref))

    def _pwr_flag(self, net, x, y):
        sym = self._power_sym("PWR_FLAG")  # ensure embedded
        self._flg_seq += 1
        ref = "#FLG%02d" % self._flg_seq
        uid = uuid_for("flg:%s:%g:%g" % (net, x, y))
        return ('\t(symbol\n'
                '\t\t(lib_id "power:PWR_FLAG")\n'
                '\t\t(at %g %g 0)\n'
                '\t\t(unit 1)\n'
                '\t\t(exclude_from_sim no) (in_bom no) (on_board yes) (dnp no)\n'
                '\t\t(uuid "%s")\n'
                '\t\t(property "Reference" "%s" (at %g %g 0)\n'
                '\t\t\t(effects (font (size 1.27 1.27)) (hide yes)))\n'
                '\t\t(property "Value" "PWR_FLAG" (at %g %g 0)\n'
                '\t\t\t(effects (font (size 1.27 1.27)) (hide yes)))\n'
                '\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))\n'
                '\t)' % (x, y, uid, ref, x, y - 5, x, y - 2,
                        self.project, self.sheet_uuid, ref))

    # ---- emit: labels / graphics ----
    @staticmethod
    def _label_orient(pin_angle):
        """Pick (text rotation, justify) so a label at the STUB END reads outward, away
        from the body. Pin angle: 0=pin on left edge, 180=right, 90=bottom, 270=top."""
        a = int(round(pin_angle)) % 360
        if a == 0:    return 0, "right"    # left-edge pin  -> text extends further left
        if a == 180:  return 0, "left"     # right-edge pin -> text extends further right
        if a == 90:   return 90, "right"   # bottom pin     -> text extends down
        return 90, "left"                  # a == 270, top pin -> text extends up

    def _local_label(self, net, x, y, key, pin_angle=0):
        rot, just = self._label_orient(pin_angle)
        return ('\t(label "%s" (at %g %g %d)\n'
                '\t\t(effects (font (size 1.27 1.27)) (justify %s))\n'
                '\t\t(uuid "%s"))' % (net, x, y, rot, just, uuid_for("lloc:" + key)))

    def _global_label(self, net, x, y, key, pin_angle=0):
        rot, just = self._label_orient(pin_angle)
        return ('\t(global_label "%s" (shape bidirectional) (at %g %g %d)\n'
                '\t\t(effects (font (size 1.27 1.27)) (justify %s))\n'
                '\t\t(uuid "%s"))' % (net, x, y, rot, just, uuid_for("lbl:" + key)))

    def _no_connect(self, x, y, key):
        return ('\t(no_connect (at %g %g) (uuid "%s"))'
                % (x, y, uuid_for("nc:" + key)))

    def _wire_seg(self, p1, p2, key):
        return ('\t(wire (pts (xy %g %g) (xy %g %g))\n'
                '\t\t(stroke (width 0) (type default))\n'
                '\t\t(uuid "%s"))' % (p1[0], p1[1], p2[0], p2[1], uuid_for("wire:" + key)))

    def _rect(self, x, y, w, h, key):
        return ('\t(rectangle (start %g %g) (end %g %g)\n'
                '\t\t(stroke (width 0.2) (type dash))\n'
                '\t\t(fill (type none))\n'
                '\t\t(uuid "%s"))' % (x, y, x + w, y + h, uuid_for("rect:" + key)))

    def _text(self, s, x, y, size, key, bold=True):
        return ('\t(text "%s" (at %g %g 0)\n'
                '\t\t(effects (font (size %g %g) %s) (justify left bottom))\n'
                '\t\t(uuid "%s"))'
                % (s, x, y, size, size, "(bold yes)" if bold else "", uuid_for("txt:" + key)))

    def _title_block(self):
        return ('\t(title_block\n'
                '\t\t(title "%s")\n'
                '\t\t(rev "%s")\n'
                '\t\t(date "%s")\n'
                '\t\t(company "%s")\n'
                '\t)' % (self.title, self.rev, self.date, self.company))

    def write(self, path):
        out = []
        out.append('(kicad_sch')
        out.append('\t(version 20250114)')
        out.append('\t(generator "kisch")')
        out.append('\t(generator_version "9.0")')
        out.append('\t(uuid "%s")' % self.sheet_uuid)
        out.append('\t(paper "%s")' % self.paper)
        out.append(self._title_block())
        # lib_symbols (unique) -- include any power glyphs we will instantiate
        seen = {}
        for c in self.comps:
            seen[c.sym.lib_id] = c.sym
        # pre-scan for power glyphs + flags
        glyphs = set()
        for (ref, pin_no), net in self.net_of.items():
            if self._is_power(net):
                g = power_glyph(net)
                if g:
                    glyphs.add(g)
        if self._flags:
            glyphs.add("PWR_FLAG")
        for g in glyphs:
            try:
                s = self._power_sym(g)
                seen[s.lib_id] = s
            except Exception:
                pass
        out.append('\t(lib_symbols')
        for sym in seen.values():
            out.append(sym.embed())
        out.append('\t)')
        # block frames + titles (drawn first so they sit behind)
        for (title, x, y, w, h) in self._blocks:
            out.append(self._rect(x, y, w, h, "%s" % title))
            out.append(self._text(title, x + 1.5, y - 0.8, 2.0, "blk:%s" % title))
        for (s, x, y, size) in self._notes:
            out.append(self._text(s, x, y, size, "note:%s:%g:%g" % (s[:8], x, y), bold=False))
        # symbol instances
        for c in self.comps:
            out.append(self._symbol_instance(c))
        # explicit wires
        for (p1, p2, key) in self._wires:
            out.append(self._wire_seg(p1, p2, key))
        # connectivity at each connected pin: a short outward STUB + a label / power glyph
        # at its end, so net names sit clear of the IC body and its pin names/numbers.
        connected_pins = set()
        for c in self.comps:
            for pin in c.sym.pins:
                net = self.net_of.get((c.ref, pin["number"]))
                pt = c.pin_point(pin)
                if not net:
                    continue
                connected_pins.add((c.ref, pin["number"]))
                key = "%s.%s" % (c.ref, pin["number"])
                ang = int(round(pin.get("angle", 0))) % 360
                ox, oy = _OUTWARD.get(ang, (-1.0, 0.0))
                end = (pt[0] + ox * STUB, pt[1] + oy * STUB)
                out.append(self._wire_seg(pt, end, key + ":stub"))
                if self._is_power(net):
                    out.append(self._power_port(net, end[0], end[1]))
                elif net in self.globals:
                    out.append(self._global_label(net, end[0], end[1], key, ang))
                else:
                    out.append(self._local_label(net, end[0], end[1], key, ang))
        # PWR_FLAGs (so supply nets are ERC-"driven")
        for (net, x, y) in self._flags:
            out.append(self._power_port(net, x, y))   # the rail/gnd glyph
            out.append(self._pwr_flag(net, x, y))      # + the flag at the same node
        # no-connect flags on unused pins of flagged refs
        for c in self.comps:
            if c.ref not in self.auto_nc:
                continue
            for pin in c.sym.pins:
                if (c.ref, pin["number"]) in connected_pins:
                    continue
                pt = c.pin_point(pin)
                out.append(self._no_connect(pt[0], pt[1], "%s.%s" % (c.ref, pin["number"])))
        out.append(')')
        open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
        self._report_collisions()
        return path

    def _report_collisions(self):
        """Print any point where pins/stub-ends of DIFFERENT nets coincide (an electrical
        short) so placement can be fixed before rendering. Returns the collision count."""
        import sys
        pts = {}   # (x,y) -> {net: "ref.pin"}
        def add(x, y, net, tag):
            k = (round(x, 2), round(y, 2))
            pts.setdefault(k, {})[net] = tag
        for c in self.comps:
            for pin in c.sym.pins:
                net = self.net_of.get((c.ref, pin["number"]))
                if not net:
                    continue
                pt = c.pin_point(pin)
                ang = int(round(pin.get("angle", 0))) % 360
                ox, oy = _OUTWARD.get(ang, (-1.0, 0.0))
                tag = "%s.%s" % (c.ref, pin["number"])
                add(pt[0], pt[1], net, tag)
                add(pt[0] + ox * STUB, pt[1] + oy * STUB, net, tag)
        bad = [(k, v) for k, v in pts.items() if len(v) > 1]
        for k, v in sorted(bad):
            sys.stderr.write("  COLLISION at %s: %s\n"
                             % (k, "  vs  ".join("%s(%s)" % (n, t) for n, t in v.items())))
        if bad:
            sys.stderr.write("  >> %d net collision(s) -- fix placement\n" % len(bad))
        return len(bad)


if __name__ == "__main__":
    # self-test: dump pins for the CAN-block parts
    for nick, name in [("Interface_CAN_LIN", "TJA1051T"),
                       ("Interface_CAN_LIN", "MCP2518FD-xQBB"),
                       ("Device", "R")]:
        s = SymDef(nick, name)
        print("%s  (%d pins)" % (s.lib_id, len(s.pins)))
        for p in s.pins:
            print("   pin %-4s %-8s at (%.2f,%.2f) a=%g len=%g"
                  % (p["number"], p["name"], p["x"], p["y"], p["angle"], p["length"]))
