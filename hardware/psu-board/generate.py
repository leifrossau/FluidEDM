"""
MS-EDM-PSU  -  EDM pulse supply (schematic-first). REVIEW-ONLY, NOT FABRICATION-READY.

> HIGH-VOLTAGE / HIGH-ENERGY. Do NOT fabricate, power, or connect to a machine until a
> qualified power-electronics engineer has reviewed the schematic, SPICE-simulated the pulse +
> sense paths, and bench-validated each stage behind current limiting. Isolated supplies,
> creepage/clearance, SiC SOA, snubber values, and ALL routing are deferred to that engineer.

Replaces the earlier PCB-only scaffold (generate_board.py) with a readable, linked
schematic + PCB built from one PARTS + NETS model, using REAL library symbols so pin
assignments are explicit and reviewable. Every HV/isolation choice is [REVIEW].
Run:  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate.py
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pcbnew
from kisch import SymDef, Comp, Schematic, uuid_for, stack_col
from kizen import Board

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 150.0, 95.0
BARRIER_X = 64.0

_SY = {}
def sym(nick, name):
    if (nick, name) not in _SY:
        _SY[(nick, name)] = SymDef(nick, name)
    return _SY[(nick, name)]

PARTS = {}
def part(ref, val, snick, sname, fplib, fpname, px, py, pr=0):
    PARTS[ref] = dict(ref=ref, val=val, sym=sym(snick, sname),
                      fplib=fplib, fp=fpname, px=px, py=py, pr=pr)

DEV = "Device"
R0805 = ("Resistor_SMD", "R_0805_2012Metric")
R2512 = ("Resistor_SMD", "R_2512_6332Metric")
C0805 = ("Capacitor_SMD", "C_0805_2012Metric")
def R(ref, val, fp, px, py): part(ref, val, DEV, "R", *fp, px, py)
def Cc(ref, val, px, py): part(ref, val, DEV, "C", *C0805, px, py)

# ============================ HV POWER REGION ============================
part("J1", "HV_DC_IN_60-120V", "Connector_Generic", "Conn_01x02", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal", 12, 18)
part("F1", "FUSE", DEV, "Fuse", "Resistor_SMD", "R_2512_6332Metric", 22, 14)
part("D10", "HV_TVS", DEV, "D_TVS", "Diode_SMD", "D_SMC", 18, 26)
part("C20", "220uF", DEV, "C_Polarized", "Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm", 30, 20)
part("C21", "220uF", DEV, "C_Polarized", "Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm", 40, 20)
part("C22", "1uF_film", DEV, "C", "Capacitor_SMD", "C_1210_3225Metric", 50, 16)
R("R20", "100k_bleed", R2512, 52, 26)
# Switch + snubber + freewheel + shunt
part("Q1", "SiC_MOSFET", "Transistor_FET", "Q_NMOS_GSD", "Package_TO_SOT_THT", "TO-247-3_Vertical", 30, 44)
R("Rg10", "10R_gate", R0805, 44, 38)
part("D11", "SiC_freewheel", DEV, "D_Schottky", "Diode_SMD", "D_SMC", 48, 44)
R("R21", "snubber_R", R2512, 56, 40); Cc("C23", "snubber_C", 56, 46)
R("R22", "5mR_shunt", R2512, 28, 60)
R("Rvd1", "1M_HVdiv", R2512, 36, 66); R("Rvd2", "3k9_HVdiv", R0805, 42, 66)
part("J2", "GAP_OUT_wire_work", "Connector_Generic", "Conn_01x02", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal", 12, 60)

# ============================ ISOLATION BARRIER ============================
part("U10", "ISO_GATE_DRIVER", "Driver_FET", "UCC21520DW", "Package_SO",
     "SOIC-16W_7.5x10.3mm_P1.27mm", BARRIER_X, 40)
part("U11", "ISO_CURRENT_AMP", "Isolator_Analog", "AMC1300BDWV", "Package_SO",
     "SOIC-8_3.9x4.9mm_P1.27mm", BARRIER_X, 58)
part("U12", "ISO_VOLTAGE_AMP", "Isolator_Analog", "AMC1300BDWV", "Package_SO",
     "SOIC-8_3.9x4.9mm_P1.27mm", BARRIER_X, 70)
part("U13", "DIGITAL_ISOLATOR", "Isolator", "ADuM1402xRW", "Package_SO",
     "SOIC-16_3.9x9.9mm_P1.27mm", BARRIER_X, 24)

# ============================ LOGIC / CAN REGION ============================
part("U1", "STM32G474CBTx", "MCU_ST_STM32G4", "STM32G474CBTx", "Package_QFP",
     "LQFP-48_7x7mm_P0.5mm", 95, 44)
part("Y1", "24MHz", DEV, "Crystal_GND24", "Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm", 80, 30)
Cc("C1", "100nF", 86, 30); Cc("C2", "100nF", 104, 30); Cc("C3", "100nF", 86, 58); Cc("C4", "10uF", 104, 58)
part("U2", "TJA1051T-3", "Interface_CAN_LIN", "TJA1051T-3", "Package_SO",
     "SOIC-8_3.9x4.9mm_P1.27mm", 120, 56)
Cc("C5", "100nF", 112, 56); R("R3", "120R", R0805, 130, 62)
part("J3", "CAN", "Connector_Generic", "Conn_01x03", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal", 134, 70)
part("J4", "LOGIC_5V_IN", "Connector_Generic", "Conn_01x02", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal", 134, 18)
part("U3", "AMS1117-3.3", "Regulator_Linear", "AMS1117-3.3", "Package_TO_SOT_SMD",
     "SOT-223-3_TabPin2", 118, 20)
Cc("C6", "10uF", 110, 26); Cc("C7", "10uF", 126, 26)
part("J5", "SWD", "Connector_Generic", "Conn_01x05", "Connector_PinHeader_2.54mm",
     "PinHeader_1x05_P2.54mm_Vertical", 80, 62)
part("J6", "NTC_TEMP", "Connector_Generic", "Conn_01x02", "TerminalBlock_Phoenix",
     "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal", 120, 80)
part("D1", "LED_RUN", DEV, "LED", "LED_SMD", "LED_0805_2012Metric", 108, 68)
R("R4", "1k", R0805, 112, 68)
part("D2", "LED_FAULT", DEV, "LED", "LED_SMD", "LED_0805_2012Metric", 108, 72)
R("R5", "1k", R0805, 112, 72)

# ============================ NETS ============================
def P(ref, spec):
    return (ref, PARTS[ref]["sym"].pin_number(spec))
NETS = {}
def net(name, *pps):
    NETS.setdefault(name, []).extend(pps)

# ---- HV power domain (separate ground HV-; [REVIEW] every value/rating) ----
net("HV+", P("J1", "1"), P("F1", "1"))
net("HVF", P("F1", "2"), P("D10", "1"), P("C20", "1"), P("C21", "1"), P("C22", "1"),
    P("R20", "1"), P("Q1", "D"))
net("HV-", P("J1", "2"), P("D10", "2"), P("C20", "2"), P("C21", "2"), P("C22", "2"),
    P("R20", "2"), P("D11", "2"), P("R22", "2"), P("C23", "2"), P("Rvd2", "2"),
    P("U10", "14"), P("U11", "4"), P("U12", "4"), P("U13", "9"), P("U13", "15"))   # iso secondaries ref to HV-
# Low-side SiC switch: HVF -> Q1.D ; Q1.S -> switch node -> GAP+ ; Q1.G via Rg10 from gate driver
net("SW_NODE", P("Q1", "S"), P("D11", "1"), P("R21", "1"), P("Rvd1", "1"), P("J2", "1"))
net("GATE", P("Q1", "G"), P("Rg10", "2"))
net("GATE_DRV", P("Rg10", "1"), ("U10", "15"))                 # UCC21520 OUTA
net("SNUB", P("R21", "2"), P("C23", "1"))
net("GAP_RTN", P("J2", "2"), P("R22", "1"))                    # gap-current shunt high side
net("VSENSE_HV", P("Rvd1", "2"), P("Rvd2", "1"), ("U12", "2")) # gap-voltage divider tap -> iso amp VINP
# isolated current sense across R22 (shunt): VINP=GAP_RTN, VINN=HV-
net("GAP_RTN", ("U11", "2")); net("HV-", ("U11", "3"))
net("HV-", ("U12", "3"))                                       # voltage-amp VINN ref to HV-

# ---- isolation barrier, LOGIC side ----
net("GATE_PWM", P("U1", "PA8"), ("U10", "1"))                  # STM32 HRTIM -> UCC21520 INA
net("ISENSE_P", ("U11", "7"), P("U1", "PA0")); net("ISENSE_N", ("U11", "6"), P("U1", "PA1"))
net("VSENSE_P", ("U12", "7"), P("U1", "PA2")); net("VSENSE_N", ("U12", "6"), P("U1", "PA3"))
net("DIO_TX", P("U1", "PB12"), ("U13", "3")); net("DIO_RX", P("U1", "PB13"), ("U13", "14"))

# ---- isolated supplies (DEFERRED isolated DC-DC; [REVIEW]) ----
net("GDRV_VDD", ("U10", "16"))                                 # UCC21520 VDDA - isolated +gate supply
net("ISO_HV_VDD", ("U11", "1"), ("U12", "1"), ("U13", "16"))   # sense-amp + digital-iso HV-side supply

# ---- logic 3V3 / 5V / GND ----
net("+5V", P("J4", "1"), P("U3", "VI"), P("C6", "1"), P("U2", "VCC"), P("C5", "1"))
net("+3V3", P("U3", "VO"), P("C7", "1"), P("U1", "1"), P("U1", "21"), P("U1", "24"),
    P("U1", "36"), P("U1", "48"), P("U1", "20"),
    P("C1", "1"), P("C2", "1"), P("C3", "1"), P("C4", "1"), P("U2", "VIO"),
    ("U10", "3"), ("U10", "8"), ("U10", "6"), ("U11", "8"), ("U12", "8"), ("U13", "1"),
    P("J5", "1"))
net("GND", P("J4", "2"), P("U3", "GND"), P("C6", "2"), P("C7", "2"),
    P("U1", "19"), P("U1", "23"), P("U1", "35"), P("U1", "47"),
    P("C1", "2"), P("C2", "2"), P("C3", "2"), P("C4", "2"),
    P("U2", "GND"), P("U2", "S"), P("C5", "2"), P("J5", "3"),
    P("R4", "2"), P("R5", "2"), P("J6", "2"), P("J3", "3"),
    ("U10", "4"), ("U10", "5"), ("U11", "5"), ("U12", "5"), ("U13", "2"), ("U13", "8"),
    P("Y1", "2"), P("Y1", "4"), P("D1", "1"), P("D2", "1"))

# ---- STM32 housekeeping ----
net("OSC_IN", P("U1", "PF0"), P("Y1", "1")); net("OSC_OUT", P("U1", "PF1"), P("Y1", "3"))
net("SWDIO", P("U1", "PA13"), P("J5", "2")); net("SWCLK", P("U1", "PA14"), P("J5", "4"))
net("NRST", P("U1", "PG10"), P("J5", "5"))            # PG10/NRST shared on LQFP48 [REVIEW]
net("FDCAN_TX", P("U1", "PB9"), P("U2", "TXD")); net("FDCAN_RX", P("U1", "PB8"), P("U2", "RXD"))
net("NTC_AIN", P("U1", "PA4"), P("J6", "1"))
net("RUN", P("U1", "PB0"), P("R4", "1")); net("RUN_LED", P("R4", "2"), P("D1", "2"))
net("FAULT", P("U1", "PB1"), P("R5", "1")); net("FAULT_LED", P("R5", "2"), P("D2", "2"))
# ---- CAN bus ----
net("CANH", P("U2", "CANH"), P("R3", "1"), P("J3", "1"))
net("CANL", P("U2", "CANL"), P("R3", "2"), P("J3", "2"))

# ============================ emit schematic ============================
S = Schematic("MS-EDM-PSU", paper="A3",
              title="MS-EDM-PSU  EDM Pulse Supply  [HV - REVIEW ONLY, NOT FAB-READY]",
              rev="A", date="2026-06-16", company="Microspark")
S.mark_global("SW_NODE", "GAP_RTN", "GATE_DRV")   # HV-spanning signal nets (no PWR_FLAG conflict)
BLOCKS = [
    ("HV INPUT + BULK  (external 60-120V isolated bus)", 10, 16, 150, 70),
    ("SiC SWITCH + SNUBBER + FREEWHEEL + SHUNT (HV)", 10, 92, 150, 110),
    ("ISOLATION BARRIER  (gate drive / I-sense / V-sense / digital)", 175, 16, 130, 200),
    ("STM32G474 CONTROLLER + FDCAN", 320, 16, 95, 130),
    ("LOGIC POWER + SWD + STATUS", 320, 152, 95, 70),
]
for (t, x, y, w, h) in BLOCKS:
    S.block(t, x, y, w, h)

place = {}
symof = lambda r: PARTS[r]["sym"]
def vcol(x, y_top, refs, gap=4.0):
    for r, yy in stack_col(refs, symof, y_top, gap).items():
        place[r] = (x, yy)

# HV input
place["J1"] = (30, 40); vcol(70, 24, ["F1", "D10"]); vcol(105, 24, ["C20", "C21", "C22", "R20"])
# Switch region
place["Q1"] = (40, 130); place["Rg10"] = (75, 110); vcol(95, 105, ["D11", "R21", "C23"])
place["R22"] = (40, 180); vcol(75, 165, ["Rvd1", "Rvd2"]); place["J2"] = (28, 150)
# Isolation barrier (stacked)
vcol(235, 30, ["U13", "U10", "U11", "U12"], gap=12.0)
# STM32 + CAN
place["U1"] = (360, 70)
vcol(330, 26, ["Y1", "C1", "C2"]); place["C3"] = (404, 30); place["C4"] = (404, 50)
place["U2"] = (360, 130); place["C5"] = (336, 130); place["R3"] = (392, 130); place["J3"] = (336, 150)
# Logic power + status
place["J4"] = (330, 160); place["U3"] = (360, 162); vcol(392, 158, ["C6", "C7"])
place["J5"] = (368, 178); place["J6"] = (396, 198)
place["D1"] = (332, 180); place["R4"] = (348, 180); place["D2"] = (332, 198); place["R5"] = (348, 198)

snap = lambda v: round(v / 1.27) * 1.27
for r, pt in PARTS.items():
    sx, sy = place.get(r, (10, 285))
    S.add(Comp(r, pt["val"], pt["sym"], "%s:%s" % (pt["fplib"], pt["fp"]), snap(sx), snap(sy)))
for name, pps in NETS.items():
    for (ref, num) in pps:
        S.connect(ref, num, name)

S.nc_unused("U1", "U10", "U11", "U12", "U13")
# Power-source flags: external HV bus, logic 5V, grounds, deferred isolated supplies [REVIEW].
S.power_flag("HV+", snap(20), snap(12)); S.power_flag("HVF", snap(55), snap(12))
S.power_flag("HV-", snap(90), snap(12)); S.power_flag("+5V", snap(330), snap(150))
S.power_flag("GND", snap(360), snap(150))
S.power_flag("GDRV_VDD", snap(175), snap(12)); S.power_flag("ISO_HV_VDD", snap(120), snap(12))

sch_path = os.path.join(HERE, "MS-EDM-PSU.kicad_sch")
S.write(sch_path)

# ============================ emit linked PCB (HV-left / logic-right, barrier) ============================
b = Board(copper_layers=4)
for r, pt in PARTS.items():
    b.add(r, pt["val"], pt["fplib"], pt["fp"], pt["px"], pt["py"], pt["pr"], uuid=uuid_for("sym:" + r))
for name, pps in NETS.items():
    for (ref, num) in pps:
        b.connect(ref, num, name, soft=True)
b.rect_outline(W, H, 0, 0, r=3.0)
b._seg((BARRIER_X, 6), (BARRIER_X, 30))
b._seg((BARRIER_X, 50), (BARRIER_X, H - 6))
b.gnd_zone(pcbnew.B_Cu, BARRIER_X - 8, H - 4, 2, 2, "HV-")
b.gnd_zone(pcbnew.B_Cu, W - (BARRIER_X + 8) - 2, H - 4, BARRIER_X + 8, 2, "GND")
b.gnd_zone(pcbnew.In2_Cu, W - (BARRIER_X + 8) - 2, H - 4, BARRIER_X + 8, 2, "GND")
for (mx, my) in [(5, 5), (W - 5, 5), (5, H - 5), (W - 5, H - 5)]:
    b.mounting_hole(mx, my, "GND")
b.fill_zones()
pcb_path = os.path.join(HERE, "MS-EDM-PSU.kicad_pcb")
b.save(pcb_path)
b.write_netlist_csv(os.path.join(HERE, "MS-EDM-PSU.netlist.csv"))
b.write_bom_csv(os.path.join(HERE, "MS-EDM-PSU.bom.csv"))
print("schematic:", sch_path)
print("pcb:", pcb_path, "|", b.summary())
if b._skipped:
    print("soft-skipped pads:", len(b._skipped))
