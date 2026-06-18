"""
MS-EDM-IFACE  -  PiBot EDM interface / expansion adapter (schematic-first, option A).

Adds to the COTS PiBot V4.96 Pro (which is fully pin-committed - see PINMAP.md) the two
front-ends the wire-EDM firmware needs and the PiBot lacks:
  * CAN-FD:  MCP2518FD controller (SPI) + TJA1051 transceiver + split termination + TVS
  * Load cell:  HX711 24-bit bridge ADC (5 V analog excitation, 3V3 digital I/O for the ESP32)
Wire-feed + tension steppers use the PiBot's OWN motor sockets 5 & 6, so NO stepper drivers here.

One source model (PARTS + NETS) emits BOTH MS-EDM-IFACE.kicad_sch (readable: power glyphs,
stubbed labels, functional blocks, NC flags) and the linked MS-EDM-IFACE.kicad_pcb.
Run:  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate.py
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pcbnew
from kisch import SymDef, Comp, Schematic, uuid_for, stack_col
from kizen import Board

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 110.0, 80.0   # board outline (mm) [REVIEW - mechanical fit vs PiBot is VERIFY-ON-HARDWARE]

# ----------------------------- symbol cache -----------------------------
_SY = {}
def sym(nick, name):
    if (nick, name) not in _SY:
        _SY[(nick, name)] = SymDef(nick, name)
    return _SY[(nick, name)]

PARTS = {}
def part(ref, val, snick, sname, fplib, fpname, px, py, pr=0):
    PARTS[ref] = dict(ref=ref, val=val, sym=sym(snick, sname),
                      fplib=fplib, fp=fpname, px=px, py=py, pr=pr)

DEV, RSMD, CSMD = "Device", "Resistor_SMD", "Capacitor_SMD"
R0805 = (RSMD, "R_0805_2012Metric"); C0805 = (CSMD, "C_0805_2012Metric")
def R(ref, val, px, py): part(ref, val, DEV, "R", *R0805, px, py)
def Cc(ref, val, px, py): part(ref, val, DEV, "C", *C0805, px, py)

# ----------------------------- parts -----------------------------
# PiBot interface headers (pin headers; flying leads to the PiBot's SPI header + screw terminals)
part("J1", "TO_PIBOT", "Connector_Generic", "Conn_01x08", "Connector_PinHeader_2.54mm",
     "PinHeader_1x08_P2.54mm_Vertical", 10, 16)            # SCK,MOSI,MISO,MCP_CS,MCP_INT,+3V3,+5V,GND
part("J2", "TO_PIBOT_BC", "Connector_Generic", "Conn_01x03", "Connector_PinHeader_2.54mm",
     "PinHeader_1x03_P2.54mm_Vertical", 10, 44)            # HX_SCK(gpio33),HX_DOUT(gpio32),GND
Cc("C1", "10uF", 24, 12)   # +3V3 bulk
Cc("C2", "10uF", 24, 48)   # +5V bulk
part("D2", "PWR", "Device", "LED", "LED_SMD", "LED_0805_2012Metric", 10, 62)
R("R1", "1k", 18, 62)

# CAN-FD controller
part("U1", "MCP2518FD", "Interface_CAN_LIN", "MCP2518FD-xQBB", "Package_SO",
     "SOIC-14_3.9x8.7mm_P1.27mm", 44, 16)
Cc("C3", "100nF", 36, 12)
part("Y1", "40MHz", "Device", "Crystal", "Crystal", "Crystal_SMD_3215-2Pin_3.2x1.5mm", 54, 26)
Cc("C4", "18pF", 50, 30); Cc("C5", "18pF", 58, 30)
# CAN-FD transceiver + termination + protection
part("U2", "TJA1051T-3", "Interface_CAN_LIN", "TJA1051T-3", "Package_SO",
     "SOIC-8_3.9x4.9mm_P1.27mm", 44, 42)
Cc("C6", "100nF", 36, 42)
# Split termination (RT1+RT2 = 120R across CANH/CANL, C7 to GND). DNP all three if this
# node is NOT a CAN bus end (see DESIGN.md).
R("RT1", "60R", 64, 36); R("RT2", "60R", 64, 42); Cc("C7", "4.7nF", 70, 39)
part("D1", "NUP2105L", "Power_Protection", "NUP2105L", "Package_TO_SOT_SMD", "SOT-23", 78, 36)
part("J3", "CAN", "Connector_Generic", "Conn_01x03", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal", 96, 38)

# Load-cell front-end (HX711)
part("U3", "HX711", "Analog_ADC", "HX711", "Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm", 52, 60)
part("Q1", "MMBT3904", "Transistor_BJT", "MMBT3904", "Package_TO_SOT_SMD", "SOT-23", 40, 58)
R("R2", "20k", 62, 56); R("R3", "8.2k", 62, 64)          # AVDD regulator divider (~4.3V)
Cc("C8", "10uF", 68, 56); Cc("C9", "100nF", 68, 60)      # AVDD decoupling
Cc("C10", "100nF", 34, 60)                                # DVDD decoupling
Cc("C11", "10uF", 34, 64)                                 # VSUP bulk (+5V)
Cc("C12", "100nF", 68, 64)                                # VBG cap
R("R4", "1k", 76, 58); R("R5", "1k", 76, 62)              # INA series (EMI)
Cc("C13", "100nF", 82, 60)                                # INA differential filter
part("J4", "LOADCELL", "Connector_Generic", "Conn_01x04", "Connector_JST",
     "JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", 96, 60)     # E+, A+, A-, E-

# ----------------------------- nets -----------------------------
def P(ref, spec):
    return (ref, PARTS[ref]["sym"].pin_number(spec))

NETS = {}
def net(name, *pps):
    NETS.setdefault(name, []).extend(pps)

# Power rails (tapped from PiBot via J1; bulk caps + LED)
net("+3V3", P("J1", "6"), P("C1", "1"), P("U1", "VDD"), P("C3", "1"),
    P("U2", "VIO"), P("U3", "DVDD"), P("C10", "1"), P("U3", "RATE"), P("R1", "1"))
net("+5V", P("J1", "7"), P("C2", "1"), P("U2", "VCC"), P("C6", "1"),
    P("U3", "VSUP"), P("Q1", "C"), P("C11", "1"))
net("GND", P("J1", "8"), P("C1", "2"), P("C2", "2"), P("U1", "7"), ("U1", "15"), P("C3", "2"),
    P("U2", "GND"), P("U2", "S"), P("C6", "2"), P("C10", "2"), P("C11", "2"),
    P("C4", "2"), P("C5", "2"), P("C7", "1"), P("D1", "3"), P("J3", "3"),
    P("J2", "3"), P("D2", "1"))
net("LED_A", P("R1", "2"), P("D2", "2"))   # +3V3 -> R1 -> LED anode -> cathode -> GND
# SPI + control to PiBot (J1)
net("SPI_SCK", P("J1", "1"), P("U1", "SCK"))
net("SPI_MOSI", P("J1", "2"), P("U1", "SDI"))
net("SPI_MISO", P("J1", "3"), P("U1", "SDO"))
net("MCP_CS", P("J1", "4"), ("U1", "13"))
net("MCP_INT", P("J1", "5"), ("U1", "4"))
# MCP2518FD oscillator
net("OSC1", ("U1", "6"), P("Y1", "1"), P("C4", "1"))
net("OSC2", ("U1", "5"), P("Y1", "2"), P("C5", "1"))
# Controller <-> transceiver
net("MCP_TXCAN", ("U1", "1"), P("U2", "TXD"))
net("MCP_RXCAN", ("U1", "2"), P("U2", "RXD"))
# CAN bus + split termination + TVS + connector
net("CANH", P("U2", "CANH"), P("RT1", "1"), ("D1", "1"), P("J3", "1"))
net("CANL", P("U2", "CANL"), P("RT2", "2"), ("D1", "2"), P("J3", "2"))
net("CAN_T", P("RT1", "2"), P("RT2", "1"), P("C7", "2"))   # split-termination centre node
# Load-cell digital to PiBot (J2 -> gpio.33/32)
net("HX_SCK", P("J2", "1"), P("U3", "PD_SCK"))
net("HX_DOUT", P("J2", "2"), P("U3", "DOUT"))
# HX711 analog supply regulator (AVDD ~4.3V from +5V via Q1; VFB divider)
net("AVDD", P("U3", "AVDD"), P("Q1", "E"), P("R2", "1"), P("C8", "1"), P("C9", "1"), P("J4", "1"))
net("HX_BASE", P("U3", "BASE"), P("Q1", "B"))
net("VFB", P("U3", "VFB"), P("R2", "2"), P("R3", "1"))
net("VBG", P("U3", "VBG"), P("C12", "1"))
# HX711 analog ground -> tied to GND (single star point at the HX711; see DESIGN.md).
# XI low selects the on-chip oscillator (XO left NC).
net("GND", P("U3", "AGND"), P("R3", "2"), P("C8", "2"), P("C9", "2"),
    P("C12", "2"), P("J4", "4"), P("U3", "XI"))
# Load-cell bridge signal (channel A) with series-R EMI filter + differential cap
net("INA_P", P("U3", "INA+"), P("R4", "2"))
net("INA_N", P("U3", "INA-"), P("R5", "2"))
net("LC_A_P", P("J4", "2"), P("R4", "1"), P("C13", "1"))
net("LC_A_N", P("J4", "3"), P("R5", "1"), P("C13", "2"))

# ----------------------------- emit schematic -----------------------------
S = Schematic("MS-EDM-IFACE", paper="A3",
              title="MS-EDM-IFACE  PiBot EDM Interface Adapter (CAN-FD + load cell)",
              rev="A", date="2026-06-16", company="Microspark")
BLOCKS = [
    ("POWER + PiBot INTERFACE  (taps SPI / limits / rails)", 10, 16, 150, 150),
    ("CAN-FD:  MCP2518FD + TJA1051 + split termination + TVS", 175, 16, 140, 170),
    ("LOAD-CELL FRONT-END:  HX711  (5V exc / 3V3 I/O)", 325, 16, 90, 184),
]
for (t, x, y, w, h) in BLOCKS:
    S.block(t, x, y, w, h)

place = {}
symof = lambda r: PARTS[r]["sym"]
def vcol(x, y_top, refs, gap=4.0):
    for r, yy in stack_col(refs, symof, y_top, gap).items():
        place[r] = (x, yy)

# POWER + interface
place["J1"] = (40, 70)
vcol(95, 24, ["C1", "C2"])
place["J2"] = (40, 140)
place["D2"] = (95, 120); place["R1"] = (120, 120)
# CAN-FD: MCP2518FD over TJA1051 (stacked); xtal + term + TVS + connector around
vcol(235, 30, ["U1", "U2"], gap=14.0)
place["C3"] = (196, 36); place["C6"] = (196, 96)
place["Y1"] = (292, 40); place["C4"] = (306, 56); place["C5"] = (306, 28)
place["RT1"] = (292, 96); place["RT2"] = (292, 120); place["C7"] = (270, 108)
place["D1"] = (250, 120); place["J3"] = (196, 150)
# HX711 front-end
place["U3"] = (360, 70)
place["Q1"] = (340, 60)
vcol(392, 30, ["R2", "R3", "C8", "C9", "C12"])
place["C10"] = (335, 120); place["C11"] = (335, 140)
vcol(392, 130, ["R4", "R5", "C13"])
place["J4"] = (340, 175)

snap = lambda v: round(v / 1.27) * 1.27
for r, pt in PARTS.items():
    sx, sy = place.get(r, (10, 280))
    S.add(Comp(r, pt["val"], pt["sym"], "%s:%s" % (pt["fplib"], pt["fp"]), snap(sx), snap(sy)))
for name, pps in NETS.items():
    for (ref, num) in pps:
        S.connect(ref, num, name)

S.nc_unused("U1", "U3")        # MCP CLKO/INT0/INT1; HX711 XO/INB+/INB-
S.power_flag("+3V3", snap(20), snap(12))
S.power_flag("+5V", snap(60), snap(12))
S.power_flag("GND", snap(100), snap(12))
S.power_flag("AVDD", snap(392), snap(150))   # AVDD is regulated by Q1 (not a power-output pin)

sch_path = os.path.join(HERE, "MS-EDM-IFACE.kicad_sch")
S.write(sch_path)

# ----------------------------- emit linked PCB -----------------------------
b = Board(copper_layers=2)
for r, pt in PARTS.items():
    b.add(r, pt["val"], pt["fplib"], pt["fp"], pt["px"], pt["py"], pt["pr"],
          uuid=uuid_for("sym:" + r))
for name, pps in NETS.items():
    for (ref, num) in pps:
        b.connect(ref, num, name, soft=True)
b.rect_outline(W, H, 0, 0, r=3.0)
b.gnd_zone(pcbnew.B_Cu, W - 4, H - 4, 2, 2, "GND")
b.gnd_zone(pcbnew.F_Cu, W - 4, H - 4, 2, 2, "GND")
for (mx, my) in [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]:
    b.mounting_hole(mx, my, "GND")
b.fill_zones()
pcb_path = os.path.join(HERE, "MS-EDM-IFACE.kicad_pcb")
b.save(pcb_path)
b.write_netlist_csv(os.path.join(HERE, "MS-EDM-IFACE.netlist.csv"))
b.write_bom_csv(os.path.join(HERE, "MS-EDM-IFACE.bom.csv"))

print("schematic:", sch_path)
print("pcb:", pcb_path, "|", b.summary())
if b._skipped:
    print("soft-skipped pads:", b._skipped)
