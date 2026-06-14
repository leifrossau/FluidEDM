"""
Generate MS-EDM-PSU.kicad_pcb — ARCHITECTURE SCAFFOLD ONLY (review, not fabrication).

  & "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" generate_board.py

HV pulse supply. Footprints are placed into an HV region (left) and an isolated logic/CAN
region (right) with a barrier gap between them. Logic/CAN side is fully netted; HV-side nets
are named for review. SiC part, snubber, isolation ratings, creepage, thermals and ALL routing
are deferred to a power engineer (see DESIGN.md). DO NOT FABRICATE OR POWER AS-IS.
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import pcbnew
from kizen import Board

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 150.0, 95.0
BARRIER_X = 64.0   # isolation slot / keep-out centre

b = Board(copper_layers=4)

R0805 = ("Resistor_SMD", "R_0805_2012Metric")
C0805 = ("Capacitor_SMD", "C_0805_2012Metric")
R2512 = ("Resistor_SMD", "R_2512_6332Metric")
SO8   = ("Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm")
TB2 = ("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal")  # PLACEHOLDER; HV/gap need uprated terminals
TB3 = ("TerminalBlock_Phoenix", "TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal")

def C(ref, lib_fp, x, y, val="", rot=0):
    b.add(ref, val, lib_fp[0], lib_fp[1], x, y, rot)

# ============================ HV POWER REGION (left) ============================
C("J1", TB2, 12, 18, "HV_DC_IN__60-120V")                         # external isolated bus
C("F1", R2512, 22, 14, "FUSE_placeholder")
C("D10", ("Diode_SMD", "D_SMC"), 18, 26, "HV_TVS")
C("C20", ("Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm"), 30, 20, "bulk_220uF")
C("C21", ("Capacitor_THT", "CP_Radial_D10.0mm_P5.00mm"), 40, 20, "bulk_220uF")
C("C22", R2512, 50, 16, "film_1uF")                               # film cap (placeholder fp)
C("R20", R2512, 52, 26, "bleeder")
C("Q1", ("Package_TO_SOT_THT", "TO-247-3_Vertical"), 30, 44, "SiC_MOSFET")
C("Rg10", R0805, 44, 38, "gate_R")
C("D11", ("Package_TO_SOT_THT", "TO-220-3_Vertical"), 48, 44, "SiC_freewheel")
C("R21", R2512, 56, 40, "snubber_R"); C("C23", C0805, 56, 46, "snubber_C")
C("R22", R2512, 28, 60, "shunt_R")                                # current sense
C("J2", TB2, 12, 60, "GAP_OUT__wire_work")

# ---- isolation barrier crossers (placed on the slot) ----
C("U10", ("Package_SO", "SOIC-16W_7.5x10.3mm_P1.27mm"), BARRIER_X, 40, "ISO_GATE_DRIVER")
C("U11", SO8, BARRIER_X, 58, "ISO_CURRENT_AMP")
C("U12", SO8, BARRIER_X, 70, "ISO_VOLTAGE_AMP")
C("U13", ("Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm"), BARRIER_X, 24, "DIGITAL_ISOLATOR")

# ============================ LOGIC / CAN REGION (right) ========================
C("U1", ("Package_QFP", "LQFP-48_7x7mm_P0.5mm"), 95, 44, "STM32G474")
C("Y1", ("Crystal", "Crystal_SMD_3225-4Pin_3.2x2.5mm"), 80, 30, "24MHz")
C("C1", C0805, 86, 30, "100nF"); C("C2", C0805, 104, 30, "100nF")
C("C3", C0805, 86, 58, "100nF"); C("C4", C0805, 104, 58, "10uF")
C("U2", SO8, 120, 56, "TJA1051")
C("C5", C0805, 112, 56, "100nF")
C("R3", R0805, 130, 62, "120R_term")
C("J3", TB3, 134, 70, "CAN")
C("J4", TB2, 134, 18, "LOGIC_5V_IN")
C("U3", ("Package_TO_SOT_SMD", "SOT-223-3_TabPin2"), 118, 20, "AMS1117-3.3")
C("C6", C0805, 110, 26, "10uF"); C("C7", C0805, 126, 26, "10uF")
C("J5", ("Connector_PinHeader_2.54mm", "PinHeader_1x05_P2.54mm_Vertical"), 80, 62, "SWD")
C("J6", TB2, 120, 80, "NTC_TEMP")
C("D1", ("LED_SMD", "LED_0805_2012Metric"), 108, 68, "LED_RUN"); C("R4", R0805, 112, 68, "1k")
C("D2", ("LED_SMD", "LED_0805_2012Metric"), 108, 72, "LED_FAULT"); C("R5", R0805, 112, 72, "1k")

# ============================ NETS ============================
def n(net, *rp): b.bus(net, *rp)

# ---- HV-side named nets (intra-region; for review, NOT routed) ----
n("HV+", ("J1","1"), ("F1","1"))
n("HVF", ("F1","2"), ("D10","1"), ("C20","1"), ("C21","1"), ("C22","1"), ("R20","1"), ("Q1","2"))  # bus after fuse (TO-247 pin2=Drain)
n("HV-", ("J1","2"), ("D10","2"), ("C20","2"), ("C21","2"), ("C22","2"), ("R20","2"),
         ("R22","2"), ("D11","2"))
n("SW_NODE", ("Q1","1"), ("Rg10","2"), ("D11","1"), ("R21","1"), ("J2","1"))   # TO-247 pin1=Gate? see note
n("GAP_RTN", ("J2","2"), ("R22","1"))
n("GATE_DRV", ("U10","9"), ("Rg10","1"))         # isolated gate driver output (verify pinout)
n("SNUB", ("R21","2"), ("C23","1")); n("SNUB_G", ("C23","2"))
n("ISENSE_HV", ("U11","2"), ("R22","1")); n("ISENSE_HV_G", ("U11","3"), ("R22","2"))
n("VSENSE_HV", ("U12","2")); n("VSENSE_HV_G", ("U12","3"))

# ---- isolation barrier: logic-side of the crossers ----
n("GATE_PWM", ("U1","26"), ("U10","2"))          # STM32 HRTIM out -> iso gate driver in (verify)
n("ISENSE", ("U11","6"), ("U1","11"))            # iso amp out -> ADC
n("VSENSE", ("U12","6"), ("U1","12"))
n("DIO_A", ("U13","2"), ("U1","27")); n("DIO_B", ("U13","15"), ("U1","28"))   # spare iso digital

# ---- logic power ----
n("+5V", ("J4","1"), ("U3","3"), ("C6","1"), ("U2","3"), ("C5","1"))
n("+3V3", ("U3","2"), ("C7","1"), ("U1","24"), ("U1","36"), ("U1","48"), ("U1","9"),
          ("C1","1"), ("C2","1"), ("C3","1"), ("C4","1"), ("U2","5"))
n("GNDL", ("J4","2"), ("U3","1"), ("C6","2"), ("C7","2"),
          ("U1","23"), ("U1","35"), ("U1","47"), ("U1","8"),
          ("C1","2"), ("C2","2"), ("C3","2"), ("C4","2"),
          ("U2","2"), ("U2","8"), ("C5","2"), ("J5","3"),
          ("R4","2"), ("R5","2"), ("J6","2"))

# ---- STM32 housekeeping (verify STM32G474 LQFP48 pinout) ----
n("OSC_IN", ("U1","5"), ("Y1","1")); n("OSC_OUT", ("U1","6"), ("Y1","3"))
n("Y1_G", ("Y1","2"), ("Y1","4"))
n("SWDIO", ("U1","34"), ("J5","2")); n("SWCLK", ("U1","37"), ("J5","4"))
n("NRST", ("U1","7"), ("J5","5")); n("SWD_3V3", ("J5","1"))
n("FDCAN_TX", ("U1","46"), ("U2","1")); n("FDCAN_RX", ("U1","45"), ("U2","4"))
n("NTC_AIN", ("U1","14"), ("J6","1"))
n("RUN", ("U1","29"), ("D1","2")); n("RUN_R", ("D1","1"), ("R4","1"))
n("FAULT", ("U1","30"), ("D2","2")); n("FAULT_R", ("D2","1"), ("R5","1"))

# ---- CAN bus ----
n("CANH", ("U2","7"), ("R3","1"), ("J3","1"))
n("CANL", ("U2","6"), ("R3","2"), ("J3","2"))
n("CAN_GND", ("J3","3"))

# ============================ OUTLINE / BARRIER / ZONES ============================
b.rect_outline(W, H, 0, 0, r=3.0)
# isolation slot: a milled gap is drawn on Edge.Cuts as two short segments (visual marker;
# a real isolation slot/route is a layout decision). Mark with silk too.
b._seg((BARRIER_X, 6), (BARRIER_X, 30))
b._seg((BARRIER_X, 50), (BARRIER_X, H - 6))
# HV ground pour (left of barrier) and logic ground pour (right of barrier)
b.gnd_zone(pcbnew.B_Cu, BARRIER_X - 8, H - 4, 2, 2, "HV-")
b.gnd_zone(pcbnew.B_Cu, W - (BARRIER_X + 8) - 2, H - 4, BARRIER_X + 8, 2, "GNDL")
b.gnd_zone(pcbnew.In2_Cu, W - (BARRIER_X + 8) - 2, H - 4, BARRIER_X + 8, 2, "GNDL")
for (mx, my) in [(5, 5), (W - 5, 5), (5, H - 5), (W - 5, H - 5)]:
    b.mounting_hole(mx, my, "GNDL")
b.fill_zones()

pcb_path = os.path.join(HERE, "MS-EDM-PSU.kicad_pcb")
ok = b.save(pcb_path)
b.write_netlist_csv(os.path.join(HERE, "MS-EDM-PSU.netlist.csv"))
b.write_bom_csv(os.path.join(HERE, "MS-EDM-PSU.bom.csv"))
print("saved:", ok, pcb_path)
print(b.summary())
