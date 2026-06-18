# Microspark EDM Interface Adapter — Board Design (MS-EDM-IFACE)

> **Status: AI-generated first-pass design. NOT fabrication-ready until reviewed.** The
> electrical net/pin map is HIGH-confidence (two agreeing vendor sources, see `PINMAP.md`); the
> **mechanical interface to the PiBot V4.96 Pro is VERIFY-ON-HARDWARE** (the vendor publishes no
> dimensioned drawing). Decisions marked **[REVIEW]** are autonomous engineering choices.

## 1. Purpose
A small adapter that gives the COTS **PiBot V4.96 Pro** FluidNC controller the two front-ends the
wire-EDM firmware (sub-project B) needs and the PiBot lacks:
- **CAN-FD node** — MCP2518FD controller (SPI) + TJA1051 transceiver, so the PiBot can speak the
  PSU + dielectric CAN-FD bus (the locked catalog: PSU `0x010–0x42F`, dielectric `0x710–0x740`).
- **Load-cell front-end** — HX711 24-bit bridge ADC for wire tension feedback (`WireFeed` PI loop).

The PiBot is **fully pin-committed** (see `PINMAP.md`); the adapter therefore **taps repurposed
pins** and is a **flying-lead / pluggable adapter**, not a stacking shield. Wire-feed + tension
steppers use the **PiBot's own motor sockets 5 & 6** — this board carries **no stepper drivers**.

## 2. Key decisions [REVIEW]
| # | Decision | Rationale | Alternative |
|---|----------|-----------|-------------|
| I1 | **MCP2518FD + TJA1051** CAN-FD front-end | Identical chip/driver to the dielectric node and the firmware's existing MCP2518FD path; classic ESP32 has no native CAN-FD | STM32 with native FDCAN (new firmware) |
| I2 | **MCP2518FD digital + HX711 DVDD on +3V3** | ESP32 is **not 5 V tolerant**; keeping MISO/DOUT at 3.3 V avoids level shifting on the shared SPI bus | level shifters (more parts/risk) |
| I3 | **HX711 analog (VSUP/AVDD) on +5V**, AVDD ≈ 4.3 V via Q1 + VFB divider | Higher bridge excitation → better load-cell SNR; DVDD stays 3.3 V so digital I/O is ESP32-safe | AVDD = 3V3 (simpler, lower SNR) |
| I4 | **CS = relay pin (gpio.26), INT = probe pin (gpio.2)** | Only repurposable outputs/inputs on a fully-committed board | gpio.13 (0-10 V) as alternate; **gpio.2 is a strapping pin — add series isolation or use alt** |
| I5 | **HX711 DOUT/SCK = unused B/C-axis limit inputs (gpio.32/33)** | Wire-EDM doesn't use B/C limits; matches the firmware config | dedicated expander |
| I6 | **Split CAN-FD termination** (2×60 Ω + 4.7 nF to GND) | Recommended for CAN-FD signal integrity at a bus end | single 120 Ω |
| I7 | **2-layer PCB** | Low pin count, no fast analog beyond the bridge; cheap | 4-layer |
| I8 | **No stepper drivers** | Wire-feed/tension use PiBot motor sockets 5 & 6 (I2S) | on-board drivers (needless) |

## 3. Block diagram
```
 PiBot SPI (gpio18/23/19) ─┬─ MCP2518FD ── TJA1051 ──┬─ CANH/CANL ─► PSU + dielectric bus
 PiBot relay/probe ─ CS/INT ┘     │  40MHz xtal       └─ split term (2×60Ω+4.7nF) + NUP2105L TVS
 PiBot +3V3/+5V/GND  ───────────────┴── power
 PiBot gpio.32/33 ── HX_DOUT/SCK ── HX711 ──┬─ Q1+VFB regulator (AVDD≈4.3V from +5V)
                                            └─ INA± (series-R + diff cap) ─► load cell (E+,A+,A-,E-)
```

## 4. Connectors
| Ref | Type | Pins | Function |
|-----|------|------|----------|
| J1 | 1×8 header | 8 | TO PIBOT: SPI_SCK, SPI_MOSI, SPI_MISO, MCP_CS, MCP_INT, +3V3, +5V, GND |
| J2 | 1×3 header | 3 | TO PIBOT (B/C limit terminals): HX_SCK (gpio.33), HX_DOUT (gpio.32), GND |
| J3 | Phoenix 3.5 mm 3P | 3 | CAN-FD bus: CANH, CANL, GND |
| J4 | JST-XH 4P | 4 | Load cell: E+ (AVDD), A+ (INA+), A− (INA−), E− (AGND) |

## 5. Power
- **+3V3** (tapped from PiBot): MCP2518FD VDD, TJA1051 VIO, HX711 DVDD, PWR LED. ~25 mA.
- **+5V** (tapped from PiBot): TJA1051 VCC, HX711 VSUP/AVDD-regulator. ~50 mA inc. CAN TX.
- **GND**: common; HX711 AGND star-tied to GND at the HX711 (single point) **[REVIEW layout]**.

## 6. Firmware ↔ hardware mapping
This board IS the PiBot's CAN-FD PHY + load-cell PHY. It carries the **entire** bus the firmware
uses (PSU `0x010/0x020/0x100/0x200–0x210/0x300/0x400–0x42F`, dielectric `0x710–0x740`). The HX711
provides `WireFeed`'s tension reading (`load_cell.dout_pin`/`sck_pin` in `wire_edm_4axis.yaml`).

## 7. Open items for review / VERIFY-ON-HARDWARE
- **Mechanical fit / connector positions / mounting** vs. the physical PiBot — not published; measure.
- **gpio.2 (CAN INT) strapping**: must be low at boot — add series isolation or move INT to gpio.13.
- **Tap points exist**: confirm SPI/3V3/5V are on an accessible header and gpio.26/2/32/33 reach terminals.
- **AVDD divider** (R2=20k / R3=8.2k → ~4.3 V): confirm against HX711 datasheet + your load-cell rating.
- **Split termination**: populate only if this node is a **bus end**; otherwise DNP RT1/RT2/C7.
- **47 Ω series R on SPI** (vendor signal-integrity note) — consider if the stub is long.
- Repo `wire_edm_4axis.yaml` tension-motor i2so bits are shifted by one — fix to STEP i2so.21 / DIR i2so.20 / DIS i2so.23 (see `PINMAP.md`).

## 8. What this board deliberately does NOT do
- No stepper drivers (PiBot sockets 5 & 6 do wire feed/tension).
- No high voltage; all rails ≤ 5 V.
- It is the PiBot's CAN PHY only — the gap servo, wire-feed logic, and protocol live in firmware.
