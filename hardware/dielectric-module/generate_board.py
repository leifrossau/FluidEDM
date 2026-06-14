"""
Generate MS-DIEL-CTRL.kicad_pcb — first-pass placement + full netlist/ratsnest.

  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate_board.py

This is a SCAFFOLD: footprints are real and placed by functional block, every net is
defined so the ratsnest is complete and routable, but tracks are NOT routed and IC pin
maps are best-effort (see DESIGN.md "[REVIEW]" + the VERIFY notes). Finalize pinouts in
the schematic against datasheets before fabrication.
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pcbnew
from kizen import Board, mm

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 130.0, 100.0

b = Board(copper_layers=4)

# ---------------- components (ref, value, lib, footprint, x, y, rot) ----------------
SOT23 = ("Package_TO_SOT_SMD", "SOT-23")
R0805 = ("Resistor_SMD", "R_0805_2012Metric")
C0805 = ("Capacitor_SMD", "C_0805_2012Metric")
DSMA  = ("Diode_SMD", "D_SMA")
TB2 = ("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal")
TB3 = ("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal")
TB6 = ("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MPT-0,5-6-2.54_1x06_P2.54mm_Horizontal")
JST4 = ("Connector_JST", "JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical")
JST6 = ("Connector_JST", "JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical")

def C(ref, lib_fp, x, y, val="", rot=0):
    b.add(ref, val, lib_fp[0], lib_fp[1], x, y, rot)

# Power input + regulation (top-left)
C("J1", TB2, 10, 12, "24V_IN")
C("Q7", SOT23, 22, 12, "DMP_revprot")          # P-FET reverse-polarity
C("D5", DSMA, 22, 20, "SMBJ24A_TVS")
C("U4", ("Package_TO_SOT_SMD", "SOT-23-6"), 36, 14, "MP2316_buck")
C("L1", ("Inductor_SMD", "L_7.3x7.3_H3.5"), 48, 14, "10uH")
C("D6", DSMA, 48, 22, "B340_cat")              # buck catch diode (if async)
C("C5", C0805, 30, 22, "22uF"); C("C6", C0805, 60, 22, "22uF")
C("R7", R0805, 28, 30, "FB_top"); C("R8", R0805, 28, 34, "FB_bot")
C("U5", ("Package_TO_SOT_SMD", "SOT-223-3_TabPin2"), 70, 14, "AMS1117-3.3")
C("C7", C0805, 64, 22, "10uF"); C("C8", C0805, 78, 22, "10uF")

# MCU (center)
C("U1", ("RF_Module", "ESP32-S3-WROOM-1"), 50, 54, "ESP32-S3-WROOM-1")
C("C1", C0805, 33, 48, "100nF"); C("C2", C0805, 33, 60, "10uF")
C("R1", R0805, 72, 46, "10k_EN"); C("C3", C0805, 76, 46, "1uF_EN")
C("R2", R0805, 72, 62, "10k_IO0"); C("C4", C0805, 76, 62, "100nF_IO0")

# CAN-FD (right)
C("U2", ("Package_SO", "SOIC-14_3.9x8.7mm_P1.27mm"), 104, 34, "MCP2518FD")
C("U3", ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm"), 104, 52, "TJA1051T/3")
C("C9", C0805, 94, 34, "100nF"); C("C10", C0805, 94, 52, "100nF")
C("R3", R0805, 116, 60, "120R_term")
C("J2", TB3, 120, 70, "CAN")

# Solenoid low-side drivers (lower-left)
C("J3", TB6, 14, 92, "FLUSH_SOL")
C("Q1", SOT23, 26, 78, "AO3400"); C("Q2", SOT23, 26, 83, "AO3400"); C("Q3", SOT23, 26, 88, "AO3400")
C("Rg1", R0805, 33, 78, "100R"); C("Rg2", R0805, 33, 83, "100R"); C("Rg3", R0805, 33, 88, "100R")
C("D1", DSMA, 18, 78, "fly1"); C("D2", DSMA, 18, 83, "fly2"); C("D3", DSMA, 18, 88, "fly3")

# SSR / contactor coil drivers (lower-mid)
C("J4", TB6, 46, 92, "PUMP_CHILL_DEION")
C("Q4", SOT23, 44, 80, "AO3400"); C("Q5", SOT23, 50, 80, "AO3400"); C("Q6", SOT23, 56, 80, "AO3400")
C("Rb4", R0805, 44, 84, "100R"); C("Rb5", R0805, 50, 84, "100R"); C("Rb6", R0805, 56, 84, "100R")

# Sensors (lower-right) + ADC dividers
C("J5", JST6, 92, 90, "ANALOG_SENSORS")       # press, level, cond(2), +5, GND
C("J6", JST4, 106, 90, "TEMP_1WIRE")
C("J7", JST6, 120, 92, "FLOW_FLOATS")
C("Rp1", R0805, 86, 78, "div_press"); C("Rp2", R0805, 86, 82, "div_press")
C("Rl1", R0805, 92, 78, "div_level"); C("Rl2", R0805, 92, 82, "div_level")
C("Rc1", R0805, 98, 78, "div_cond");  C("Rc2", R0805, 98, 82, "div_cond")
C("R4", R0805, 104, 72, "4k7_1wire_pullup")

# USB-C (bottom edge)
C("J8", ("Connector_USB", "USB_C_Receptacle_Amphenol_12401548E4-2A"), 70, 94, "USB-C")
C("R5", R0805, 62, 88, "5k1_CC1"); C("R6", R0805, 78, 88, "5k1_CC2")

# ---------------- nets ----------------
def n(net, *rp): b.bus(net, *rp)

# Power tree
n("24V_RAW", ("J1","1"))
n("+24V", ("Q7","3"), ("D5","2"), ("U4","2"), ("D6","1"),
          ("D1","1"), ("D2","1"), ("D3","1"), ("C5","1"))
n("+5V", ("L1","2"), ("C6","1"), ("R7","1"), ("U5","3"), ("C7","1"),
         ("U3","3"), ("J5","5"), ("J7","1"), ("J8","A4"), ("J8","B4"), ("J8","A9"), ("J8","B9"))
n("SW5", ("U4","1"), ("L1","1"), ("D6","2"))     # buck switch node
n("FB5", ("U4","4"), ("R7","2"), ("R8","1"))
n("+3V3", ("U5","2"), ("C8","1"), ("U1","2"), ("C1","1"), ("U2","13"),
          ("C9","1"), ("U3","5"), ("C10","1"), ("C2","1"), ("R1","1"), ("R4","1"), ("J6","1"))
n("GND", ("J1","2"), ("Q7","2"), ("D5","1"), ("U4","3"), ("C5","2"), ("C6","2"),
         ("R8","2"), ("U5","1"), ("C7","2"), ("C8","2"),
         ("U1","1"), ("U1","40"), ("U1","41"), ("C1","2"), ("C2","2"), ("C3","2"), ("C4","2"),
         ("U2","14"), ("C9","2"), ("U3","2"), ("U3","8"), ("C10","2"),
         ("Q1","2"), ("Q2","2"), ("Q3","2"), ("Q4","2"), ("Q5","2"), ("Q6","2"),
         ("J5","6"), ("J6","3"), ("J7","6"), ("J3","6"), ("J4","6"),
         ("J8","A1"), ("J8","B1"), ("J8","A12"), ("J8","B12"), ("J8","SH"))

# EN / boot
n("EN", ("U1","3"), ("R1","2"), ("C3","1"))
n("IO0", ("U1","27"), ("R2","1"), ("C4","1")); n("IO0R", ("R2","2"))

# SPI: ESP32 <-> MCP2518FD   (VERIFY MCP2518FD 14-SOIC pinout against datasheet)
n("SPI_SCK",  ("U1","20"), ("U2","4"))
n("SPI_MOSI", ("U1","19"), ("U2","5"))
n("SPI_MISO", ("U1","21"), ("U2","6"))
n("MCP_CS",   ("U1","18"), ("U2","7"))
n("MCP_INT",  ("U1","17"), ("U2","3"))
# MCP <-> transceiver
n("MCP_TXCAN", ("U2","10"), ("U3","1"))
n("MCP_RXCAN", ("U2","11"), ("U3","4"))
n("OSC1", ("U2","1")); n("OSC2", ("U2","2"))     # crystal/osc TBD (see DESIGN)
# CAN bus
n("CANH", ("U3","7"), ("R3","1"), ("J2","1"))
n("CANL", ("U3","6"), ("R3","2"), ("J2","2"))
n("CAN_GND", ("J2","3"))

# USB (native): D-/D+ to IO19/IO20
n("USB_DM", ("U1","13"), ("J8","A7"), ("J8","B7"))
n("USB_DP", ("U1","14"), ("J8","A6"), ("J8","B6"))
n("CC1", ("J8","A5"), ("R5","1")); n("CC1G", ("R5","2"))
n("CC2", ("J8","B5"), ("R6","1")); n("CC2G", ("R6","2"))

# Sensors -> MCU
n("TEMP_1W", ("U1","4"), ("R4","2"), ("J6","2"))
n("FLOW_PULSE", ("U1","5"), ("J7","2"))
n("FLOAT_LO", ("U1","6"), ("J7","3"))
n("FLOAT_HI", ("U1","7"), ("J7","4"))
n("COND_DRV", ("U1","12"), ("J5","3"))
n("PRESS_AIN", ("U1","39"), ("Rp1","2"), ("Rp2","1")); n("PRESS_RAW", ("J5","1"), ("Rp1","1")); n("PRESS_GND", ("Rp2","2"))
n("LEVEL_AIN", ("U1","38"), ("Rl1","2"), ("Rl2","1")); n("LEVEL_RAW", ("J5","2"), ("Rl1","1")); n("LEVEL_GND", ("Rl2","2"))
n("COND_AIN",  ("U1","15"), ("Rc1","2"), ("Rc2","1")); n("COND_RAW",  ("J5","4"), ("Rc1","1")); n("COND_GND", ("Rc2","2"))

# Drives: solenoids (low-side) -> J3, with flyback to +24V
n("VLV_LOW",  ("U1","8"),  ("Rg1","1")); n("VLV_MED", ("U1","9"), ("Rg2","1")); n("VLV_HIGH", ("U1","10"), ("Rg3","1"))
n("G1", ("Rg1","2"), ("Q1","1")); n("G2", ("Rg2","2"), ("Q2","1")); n("G3", ("Rg3","2"), ("Q3","1"))
n("SOL1", ("Q1","3"), ("D1","2"), ("J3","1"))
n("SOL2", ("Q2","3"), ("D2","2"), ("J3","2"))
n("SOL3", ("Q3","3"), ("D3","2"), ("J3","3"))
n("J3_24a", ("J3","4")); n("J3_24b", ("J3","5"))

# Drives: SSR/contactor coils -> J4
n("SSR_PUMP",  ("U1","11"), ("Rb4","1")); n("SSR_CHILL", ("U1","23"), ("Rb5","1")); n("SSR_DEION", ("U1","24"), ("Rb6","1"))
n("Gb4", ("Rb4","2"), ("Q4","1")); n("Gb5", ("Rb5","2"), ("Q5","1")); n("Gb6", ("Rb6","2"), ("Q6","1"))
n("SSR_PUMP_O", ("Q4","3"), ("J4","1")); n("SSR_CHILL_O", ("Q5","3"), ("J4","2")); n("SSR_DEION_O", ("Q6","3"), ("J4","3"))
n("J4_x", ("J4","4")); n("J4_y", ("J4","5"))

# ---------------- board outline, zones, mounting ----------------
b.rect_outline(W, H, 0, 0, r=3.0)
b.gnd_zone(pcbnew.In2_Cu, W - 4, H - 4, 2, 2, "GND")
b.gnd_zone(pcbnew.B_Cu,  W - 4, H - 4, 2, 2, "GND")
for (mx, my) in [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]:
    b.mounting_hole(mx, my, "GND")
b.fill_zones()

pcb_path = os.path.join(HERE, "MS-DIEL-CTRL.kicad_pcb")
ok = b.save(pcb_path)
b.write_netlist_csv(os.path.join(HERE, "MS-DIEL-CTRL.netlist.csv"))
b.write_bom_csv(os.path.join(HERE, "MS-DIEL-CTRL.bom.csv"))
print("saved:", ok, pcb_path)
print(b.summary())
