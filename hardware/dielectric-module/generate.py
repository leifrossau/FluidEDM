"""
MS-DIEL-CTRL — schematic-first (option A) generator.

One shared source model (PARTS + NETS) emits BOTH:
  - MS-DIEL-CTRL.kicad_sch  (source of truth; connectivity by global-label-per-pin)
  - MS-DIEL-CTRL.kicad_pcb  (footprints linked to the schematic symbols by shared UUID)

ICs are wired by PIN NAME (resolved against the real library symbol), the ESP32 by GPIO
name, so the netlist uses authoritative pinouts. Run with the KiCad-bundled python:
  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate.py
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pcbnew
from kisch import SymDef, Comp, Schematic, uuid_for, stack_col
from kizen import Board

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 130.0, 100.0

# ----------------------------- symbol cache -----------------------------
_SY = {}
def sym(nick, name):
    if (nick, name) not in _SY:
        _SY[(nick, name)] = SymDef(nick, name)
    return _SY[(nick, name)]

# ----------------------------- parts -----------------------------
PARTS = {}
def part(ref, val, snick, sname, fplib, fpname, px, py, pr=0):
    PARTS[ref] = dict(ref=ref, val=val, sym=sym(snick, sname),
                      fplib=fplib, fp=fpname, px=px, py=py, pr=pr)

DEV, RSMD, CSMD = "Device", "Resistor_SMD", "Capacitor_SMD"
SOT, SO = "Package_TO_SOT_SMD", "Package_SO"
R0805 = (RSMD, "R_0805_2012Metric"); C0805 = (CSMD, "C_0805_2012Metric")
DSMA = ("Diode_SMD", "D_SMA")
def R(ref, val, px, py): part(ref, val, DEV, "R", *R0805, px, py)
def Cc(ref, val, px, py): part(ref, val, DEV, "C", *C0805, px, py)
def Dd(ref, val, px, py, sch="D"): part(ref, val, DEV, sch, *DSMA, px, py)
def Nfet(ref, val, px, py): part(ref, val, "Transistor_FET", "AO3400A", SOT, "SOT-23", px, py)

# MCU
part("U1", "ESP32-S3-WROOM-1", "RF_Module", "ESP32-S3-WROOM-1", "RF_Module", "ESP32-S3-WROOM-1", 50, 54)
Cc("C1", "100nF", 33, 48); Cc("C2", "10uF", 33, 60)
R("R1", "10k", 72, 46); Cc("C3", "1uF", 76, 46)
R("R2", "10k", 72, 62); Cc("C4", "100nF", 76, 62)
# Power
part("J1", "24V_IN", "Connector_Generic", "Conn_01x02", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal", 10, 12)
part("Q7", "AO3401A", "Transistor_FET", "AO3401A", SOT, "SOT-23", 22, 12)
Dd("D5", "SMBJ24A", 18, 20)
part("U4", "AP3402", "Regulator_Switching", "AP3402", SOT, "SOT-23-6", 36, 14)
part("L1", "10uH", DEV, "L", "Inductor_SMD", "L_7.3x7.3_H3.5", 48, 14)
Dd("D6", "B340", 48, 22, sch="D_Schottky")
Cc("C5", "22uF", 30, 22); Cc("C6", "22uF", 60, 22)
R("R7", "FB_top", 28, 30); R("R8", "FB_bot", 28, 34)
part("U5", "AMS1117-3.3", "Regulator_Linear", "AMS1117-3.3", SOT, "SOT-223-3_TabPin2", 70, 14)
Cc("C7", "10uF", 64, 22); Cc("C8", "10uF", 78, 22)
# CAN
part("U2", "MCP2518FD", "Interface_CAN_LIN", "MCP2518FD-xQBB", SO, "SOIC-14_3.9x8.7mm_P1.27mm", 104, 34)
part("U3", "TJA1051T-3", "Interface_CAN_LIN", "TJA1051T-3", SO, "SOIC-8_3.9x4.9mm_P1.27mm", 104, 52)
Cc("C9", "100nF", 94, 34); Cc("C10", "100nF", 94, 52)
R("R3", "120R", 116, 60)
# MCP2518FD oscillator: 40 MHz crystal + load caps (Pierce). [REVIEW] cap value vs crystal CL.
part("Y1", "40MHz", "Device", "Crystal", "Crystal", "Crystal_SMD_3215-2Pin_3.2x1.5mm", 112, 40)
Cc("C11", "18pF", 108, 44); Cc("C12", "18pF", 116, 44)
part("J2", "CAN", "Connector_Generic", "Conn_01x03", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal", 120, 70)
# Solenoid drivers
part("J3", "FLUSH_SOL", "Connector_Generic", "Conn_01x06", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal", 14, 92)
Nfet("Q1", "AO3400A", 26, 78); Nfet("Q2", "AO3400A", 26, 83); Nfet("Q3", "AO3400A", 26, 88)
R("Rg1", "100R", 33, 78); R("Rg2", "100R", 33, 83); R("Rg3", "100R", 33, 88)
Dd("D1", "fly", 18, 78); Dd("D2", "fly", 18, 83); Dd("D3", "fly", 18, 88)
# SSR drivers
part("J4", "PUMP_CHILL_DEION", "Connector_Generic", "Conn_01x06", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal", 46, 92)
Nfet("Q4", "AO3400A", 44, 80); Nfet("Q5", "AO3400A", 50, 80); Nfet("Q6", "AO3400A", 56, 80)
R("Rb4", "100R", 44, 84); R("Rb5", "100R", 50, 84); R("Rb6", "100R", 56, 84)
# Sensors
part("J5", "ANALOG_SENSORS", "Connector_Generic", "Conn_01x06", "Connector_JST",
     "JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 92, 90)
part("J6", "TEMP_1WIRE", "Connector_Generic", "Conn_01x04", "Connector_JST",
     "JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", 106, 90)
part("J7", "FLOW_FLOATS", "Connector_Generic", "Conn_01x06", "Connector_JST",
     "JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 120, 92)
R("Rp1", "div", 86, 78); R("Rp2", "div", 86, 82)
R("Rl1", "div", 92, 78); R("Rl2", "div", 92, 82)
R("Rc1", "div", 98, 78); R("Rc2", "div", 98, 82)
R("R4", "4k7", 104, 72)
# Programming / power header (replaces USB-C; see DESIGN [REVIEW])
part("J8", "PROG_USB", "Connector_Generic", "Conn_01x06", "Connector_PinHeader_2.54mm",
     "PinHeader_1x06_P2.54mm_Vertical", 70, 94)

# ----------------------------- nets -----------------------------
def P(ref, spec):
    return (ref, PARTS[ref]["sym"].pin_number(spec))

NETS = {}
def net(name, *pps):
    NETS.setdefault(name, []).extend(pps)

net("24V_RAW", P("J1", "1"), P("Q7", "S"))
net("+24V", P("Q7", "D"), P("D5", "1"), P("U4", "VIN"), P("U4", "EN"), P("D6", "1"),
    P("D1", "1"), P("D2", "1"), P("D3", "1"), P("C5", "1"),
    P("J3", "4"), P("J3", "5"))
net("SW", P("U4", "LX"), P("L1", "1"), P("D6", "2"))
net("FB", P("U4", "FB"), P("R7", "2"), P("R8", "1"))
net("+5V", P("L1", "2"), P("C6", "1"), P("R7", "1"), P("U5", "VI"), P("C7", "1"),
    P("U3", "VCC"), P("J5", "5"), P("J7", "1"), P("J8", "1"))
net("+3V3", P("U5", "VO"), P("C8", "1"), P("U1", "3V3"), P("C1", "1"), P("U2", "VDD"),
    P("C9", "1"), P("U3", "VIO"), P("C10", "1"), P("C2", "1"), P("R1", "1"), P("R2", "2"),
    P("R4", "1"), P("J6", "1"))
net("GND", P("J1", "2"), P("Q7", "G"), P("D5", "2"), P("U4", "GND"), P("C5", "2"), P("C6", "2"),
    P("R8", "2"), P("U5", "GND"), P("C7", "2"), P("C8", "2"),
    P("U1", "1"), P("U1", "40"), P("U1", "41"), P("C1", "2"), P("C2", "2"), P("C3", "2"), P("C4", "2"),
    P("U2", "7"), P("U2", "15"), P("C9", "2"), P("U3", "GND"), P("U3", "S"), P("C10", "2"),
    P("Q1", "S"), P("Q2", "S"), P("Q3", "S"), P("Q4", "S"), P("Q5", "S"), P("Q6", "S"),
    P("J2", "3"), P("J5", "6"), P("J6", "3"), P("J7", "6"), P("J3", "6"), P("J4", "6"),
    P("J4", "4"), P("J4", "5"), P("J8", "2"),
    P("Rp2", "2"), P("Rl2", "2"), P("Rc2", "2"))
# EN / IO0
net("EN", P("U1", "EN"), P("R1", "2"), P("C3", "1"), P("J8", "5"))
net("IO0", P("U1", "IO0"), P("R2", "1"), P("C4", "1"), P("J8", "6"))
# SPI + CAN controller (real MCP pinout)
net("SPI_SCK", P("U1", "IO12"), P("U2", "SCK"))
net("SPI_MOSI", P("U1", "IO11"), P("U2", "SDI"))
net("SPI_MISO", P("U1", "IO13"), P("U2", "SDO"))
net("MCP_CS", P("U1", "IO10"), ("U2", "13"))
net("MCP_INT", P("U1", "IO9"), ("U2", "4"))
net("MCP_TXCAN", ("U2", "1"), P("U3", "TXD"))
net("MCP_RXCAN", ("U2", "2"), P("U3", "RXD"))
net("OSC1", ("U2", "6"), P("Y1", "1"), P("C11", "1"))
net("OSC2", ("U2", "5"), P("Y1", "2"), P("C12", "1"))
net("GND", P("C11", "2"), P("C12", "2"))   # crystal load-cap returns
# CAN bus
net("CANH", P("U3", "CANH"), P("R3", "1"), P("J2", "1"))
net("CANL", P("U3", "CANL"), P("R3", "2"), P("J2", "2"))
# USB (native) to header
net("USB_DP", P("U1", "USB_D+"), P("J8", "3"))
net("USB_DM", P("U1", "USB_D-"), P("J8", "4"))
# sensors
net("TEMP_1W", P("U1", "IO4"), P("R4", "2"), P("J6", "2"))
net("FLOW_PULSE", P("U1", "IO5"), P("J7", "2"))
net("FLOAT_LO", P("U1", "IO6"), P("J7", "3"))
net("FLOAT_HI", P("U1", "IO7"), P("J7", "4"))
net("COND_DRV", P("U1", "IO8"), P("J5", "3"))
net("PRESS_AIN", P("U1", "IO1"), P("Rp1", "2"), P("Rp2", "1"))
net("PRESS_RAW", P("J5", "1"), P("Rp1", "1"))
net("LEVEL_AIN", P("U1", "IO2"), P("Rl1", "2"), P("Rl2", "1"))
net("LEVEL_RAW", P("J5", "2"), P("Rl1", "1"))
net("COND_AIN", P("U1", "IO3"), P("Rc1", "2"), P("Rc2", "1"))
net("COND_RAW", P("J5", "4"), P("Rc1", "1"))
# solenoid drives
net("VLV_LOW", P("U1", "IO15"), P("Rg1", "1"))
net("VLV_MED", P("U1", "IO16"), P("Rg2", "1"))
net("VLV_HIGH", P("U1", "IO17"), P("Rg3", "1"))
net("G1", P("Rg1", "2"), P("Q1", "G")); net("G2", P("Rg2", "2"), P("Q2", "G")); net("G3", P("Rg3", "2"), P("Q3", "G"))
net("SOL1", P("Q1", "D"), P("D1", "2"), P("J3", "1"))
net("SOL2", P("Q2", "D"), P("D2", "2"), P("J3", "2"))
net("SOL3", P("Q3", "D"), P("D3", "2"), P("J3", "3"))
# SSR drives
net("SSR_PUMP", P("U1", "IO18"), P("Rb4", "1"))
net("SSR_CHILL", P("U1", "IO21"), P("Rb5", "1"))
net("SSR_DEION", P("U1", "IO47"), P("Rb6", "1"))
net("Gb4", P("Rb4", "2"), P("Q4", "G")); net("Gb5", P("Rb5", "2"), P("Q5", "G")); net("Gb6", P("Rb6", "2"), P("Q6", "G"))
net("SSR_PUMP_O", P("Q4", "D"), P("J4", "1"))
net("SSR_CHILL_O", P("Q5", "D"), P("J4", "2"))
net("SSR_DEION_O", P("Q6", "D"), P("J4", "3"))

# ----------------------------- emit schematic (functional blocks) -----------------------------
S = Schematic("MS-DIEL-CTRL", paper="A3",
              title="MS-DIEL-CTRL  Dielectric / Coolant CAN-FD Controller",
              rev="A", date="2026-06-16", company="Microspark")
order = list(PARTS.values())

# Functional block frames (x, y, w, h) on the A3 sheet (420 x 297 mm).
BLOCKS = [
    ("POWER:  24V in  ->  5V buck  ->  3V3 LDO", 10, 16, 158, 96),
    ("ESP32-S3 MCU  +  PROGRAM / USB HEADER",   176, 16, 104, 168),
    ("CAN-FD:  MCP2518FD  +  TJA1051 xcvr",      292, 16, 116, 96),
    ("FLUSH SOLENOID DRIVERS  (low-side N-FET)", 10, 124, 158, 120),
    ("PUMP / CHILLER / DEIONISER  (SSR drive)", 176, 196, 104, 60),
    ("ANALOG + DIGITAL SENSORS",                 292, 124, 116, 120),
]
for (t, x, y, w, h) in BLOCKS:
    S.block(t, x, y, w, h)

# Explicit per-part placement inside the blocks (symbol-origin x,y).
# Vertical columns use stack_col() which auto-spaces by each part's pin extent so facing
# pin-stubs never collide (verified by the generator's collision report).
place = {}
symof = lambda r: PARTS[r]["sym"]
def vcol(x, y_top, refs, gap=3.81):
    for r, yy in stack_col(refs, symof, y_top, gap).items():
        place[r] = (x, yy)

# POWER:  input protection | buck | feedback | LDO
vcol(28,  24, ["J1", "Q7", "D5"])
vcol(72,  24, ["U4", "L1", "D6", "C5"])
vcol(116, 22, ["C6", "R7", "R8"])
vcol(150, 22, ["U5", "C7", "C8"])
# MCU:  U1 (large) centred; support R/C column to the right; program header below
place["U1"] = (214, 96)
vcol(264, 22, ["C1", "C2", "R1", "C3", "R2", "C4"])
place["J8"] = (250, 168)
# CAN:  MCP2518FD over TJA1051 (stacked so their side labels don't clash); xtal + caps around
vcol(338, 26, ["U2", "U3"], gap=10.0)
place["C9"] = (300, 40); place["C10"] = (300, 92)         # decoupling, left of the ICs
place["Y1"] = (388, 44); place["C11"] = (402, 60); place["C12"] = (402, 36)  # xtal, right
place["R3"] = (388, 92); place["J2"] = (300, 70)
# FLUSH solenoids:  flyback diodes | FETs | gate resistors | connector
vcol(28,  134, ["D1", "D2", "D3"], gap=8.0)
vcol(66,  134, ["Q1", "Q2", "Q3"], gap=8.0)
vcol(104, 134, ["Rg1", "Rg2", "Rg3"], gap=8.0)
place["J3"] = (142, 150)
# SSR drivers:  FET row, gate-resistor row, connector
place["Q4"] = (192, 212); place["Q5"] = (214, 212); place["Q6"] = (236, 212)
place["Rb4"] = (192, 236); place["Rb5"] = (214, 236); place["Rb6"] = (236, 236)
place["J4"] = (262, 222)
# SENSORS:  divider pairs (pressure/level/cond) in two columns + 1-Wire pull-up; connectors
vcol(304, 132, ["Rp1", "Rp2", "Rl1", "Rl2"])
vcol(330, 132, ["Rc1", "Rc2", "R4"])
place["J5"] = (360, 150); place["J6"] = (360, 200); place["J7"] = (392, 150)

def snap(v):
    return round(v / 1.27) * 1.27   # land symbol origins (=> pins, labels) on the 1.27 mm grid

for pt in order:
    sx, sy = place.get(pt["ref"], (10, 280))   # fallback: bottom-left margin
    fpfield = "%s:%s" % (pt["fplib"], pt["fp"])
    S.add(Comp(pt["ref"], pt["val"], pt["sym"], fpfield, snap(sx), snap(sy)))
for name, pps in NETS.items():
    for (ref, num) in pps:
        S.connect(ref, num, name)

# No-connect flags on genuinely-unused IC + connector spare pins (kills ERC pin-not-connected):
#   U1 unused GPIO, U2 MCP2518FD CLKO/INT0/INT1, U4 buck PG, J6/J7 spare pins.
S.nc_unused("U1", "U2", "U4", "J6", "J7")
# PWR_FLAG only on rails with NO power-output pin driving them (+3V3 is driven by U5's VO).
S.power_flag("+24V", snap(40), snap(12))
S.power_flag("+5V", snap(80), snap(12))
S.power_flag("GND", snap(120), snap(12))

sch_path = os.path.join(HERE, "MS-DIEL-CTRL.kicad_sch")
S.write(sch_path)

# ----------------------------- emit linked PCB -----------------------------
b = Board(copper_layers=4)
for pt in PARTS.values():
    b.add(pt["ref"], pt["val"], pt["fplib"], pt["fp"], pt["px"], pt["py"], pt["pr"],
          uuid=uuid_for("sym:" + pt["ref"]))
for name, pps in NETS.items():
    for (ref, num) in pps:
        b.connect(ref, num, name, soft=True)
b.rect_outline(W, H, 0, 0, r=3.0)
b.gnd_zone(pcbnew.In2_Cu, W - 4, H - 4, 2, 2, "GND")
b.gnd_zone(pcbnew.B_Cu, W - 4, H - 4, 2, 2, "GND")
for (mx, my) in [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]:
    b.mounting_hole(mx, my, "GND")
b.fill_zones()
pcb_path = os.path.join(HERE, "MS-DIEL-CTRL.kicad_pcb")
b.save(pcb_path)
b.write_netlist_csv(os.path.join(HERE, "MS-DIEL-CTRL.netlist.csv"))
b.write_bom_csv(os.path.join(HERE, "MS-DIEL-CTRL.bom.csv"))

print("schematic:", sch_path)
print("pcb:", pcb_path, "|", b.summary())
if b._skipped:
    print("soft-skipped pads (no footprint pad, e.g. exposed pad):", b._skipped)
