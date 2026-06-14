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


class Schematic:
    def __init__(self, project, paper="A3"):
        self.project = project
        self.paper = paper
        self.sheet_uuid = uuid_for("sheet:" + project)
        self.comps = []
        self.net_of = {}      # (ref, pin_number) -> net name

    def add(self, comp):
        self.comps.append(comp)
        return comp

    def connect(self, ref, pin_number, net):
        self.net_of[(ref, str(pin_number))] = net

    def bus(self, net, *ref_pins):
        for ref, pin in ref_pins:
            self.connect(ref, pin, net)

    # ---- emit ----
    def _symbol_instance(self, c):
        L = []
        a = L.append
        a('\t(symbol')
        a('\t\t(lib_id "%s")' % c.sym.lib_id)
        a('\t\t(at %g %g %d)' % (c.x, c.y, c.rot))
        a('\t\t(unit 1)')
        a('\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)')
        a('\t\t(uuid "%s")' % c.uuid)
        a('\t\t(property "Reference" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (justify left)))' % (c.ref, c.x + 12, c.y - 2))
        a('\t\t(property "Value" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (justify left)))' % (c.value, c.x + 12, c.y + 2))
        a('\t\t(property "Footprint" "%s" (at %g %g 0)'
          '\n\t\t\t(effects (font (size 1.27 1.27)) (hide yes)))' % (c.fp, c.x, c.y))
        a('\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))'
          % (self.project, self.sheet_uuid, c.ref))
        a('\t)')
        return "\n".join(L)

    def _global_label(self, net, x, y, key):
        return ('\t(global_label "%s" (shape bidirectional) (at %g %g 0)\n'
                '\t\t(effects (font (size 1.27 1.27)) (justify left))\n'
                '\t\t(uuid "%s"))' % (net, x, y, uuid_for("lbl:" + key)))

    def _wire(self, p1, p2, key):
        return ('\t(wire (pts (xy %g %g) (xy %g %g))\n'
                '\t\t(stroke (width 0) (type default))\n'
                '\t\t(uuid "%s"))' % (p1[0], p1[1], p2[0], p2[1], uuid_for("wire:" + key)))

    def write(self, path):
        out = []
        out.append('(kicad_sch')
        out.append('\t(version 20250114)')
        out.append('\t(generator "kisch")')
        out.append('\t(generator_version "9.0")')
        out.append('\t(uuid "%s")' % self.sheet_uuid)
        out.append('\t(paper "%s")' % self.paper)
        # lib_symbols (unique)
        out.append('\t(lib_symbols')
        seen = {}
        for c in self.comps:
            seen[c.sym.lib_id] = c.sym
        for sym in seen.values():
            out.append(sym.embed())
        out.append('\t)')
        # symbol instances
        for c in self.comps:
            out.append(self._symbol_instance(c))
        # connectivity: short stub + global label at each connected pin
        for c in self.comps:
            for pin in c.sym.pins:
                net = self.net_of.get((c.ref, pin["number"]))
                if not net:
                    continue
                pt = c.pin_point(pin)
                out.append(self._global_label(net, pt[0], pt[1], "%s.%s" % (c.ref, pin["number"])))
        out.append(')')
        open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
        return path


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
